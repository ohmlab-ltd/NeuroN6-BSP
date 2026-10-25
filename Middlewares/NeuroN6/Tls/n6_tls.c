/**
  ******************************************************************************
  * @file    n6_tls.c
  * @brief   TLS over Wi-Fi: entropy, mutexes, time, the board's identity and
  *          a client connection. See n6_tls.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  ******************************************************************************
  */

#include "n6_tls.h"

#include "stm32n6xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "mbedtls/entropy.h"
#include "mbedtls/error.h"
#include "mbedtls/oid.h"
#include "mbedtls/pem.h"
#include "mbedtls/pk.h"
#include "mbedtls/platform_time.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/psa_util.h"
#include "mbedtls/sha256.h"
#include "mbedtls/threading.h"
#include "mbedtls/x509_crt.h"
#include "psa/crypto.h"

/* NXP's Mbed TLS transport over lwIP's sockets (wm_mbedtls_net.c). */
#include "wm_mbedtls_net.h"

#include <stdio.h>
#include <string.h>

/* Entropy callback (MBEDTLS_ENTROPY_HARDWARE_ALT); its prototype is in a
 * library-private header. */
int mbedtls_hardware_poll(void *pData, unsigned char *pOut, size_t Len, size_t *pOutLen);

/* State */

static volatile uint8_t  Ready;
static RNG_HandleTypeDef Rng;
static SemaphoreHandle_t RngLock;
static StaticSemaphore_t RngLockStorage;

/* The wall clock: Unix seconds at the tick TimeTick, 0 while unknown. */
static uint32_t TimeBase;
static uint32_t TimeTick;

/* The tick extended to 64 bits, for mbedtls_ms_time(). */
static uint32_t MsLast;
static uint32_t MsHigh;

/* Both clocks advance only when read, and would miss a 32-bit tick wrap
 * (49.7 days) if unread that long; this timer reads them hourly. */
#define CLOCK_FOLD_MS  3600000U
static StaticTimer_t ClockTimerStorage;
static TimerHandle_t ClockTimer;

/* Entropy */

/* One word from the RNG. A seed or clock error leaves the HAL handle locked:
 * restart once, discarding the first word as at start-up. */
static int rng_word(uint32_t *pWord)
{
  if (HAL_RNG_GenerateRandomNumber(&Rng, pWord) == HAL_OK)
  {
    return 0;
  }
  (void)HAL_RNG_DeInit(&Rng);
  if ((HAL_RNG_Init(&Rng) != HAL_OK) ||
      (HAL_RNG_GenerateRandomNumber(&Rng, pWord) != HAL_OK) ||
      (HAL_RNG_GenerateRandomNumber(&Rng, pWord) != HAL_OK))
  {
    return -1;
  }
  return 0;
}

int mbedtls_hardware_poll(void *pData, unsigned char *pOut, size_t Len, size_t *pOutLen)
{
  uint32_t word = 0U;
  size_t   done = 0U;
  int      ret  = 0;

  (void)pData;
  *pOutLen = 0U;
  if ((Rng.Instance == NULL) || (RngLock == NULL))
  {
    return MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
  }
  (void)xSemaphoreTake(RngLock, portMAX_DELAY);
  while (done < Len)
  {
    const size_t n = ((Len - done) < sizeof(word)) ? (Len - done) : sizeof(word);

    if (rng_word(&word) != 0)
    {
      ret = MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
      break;
    }
    memcpy(&pOut[done], &word, n);
    done += n;
  }
  (void)xSemaphoreGive(RngLock);
  mbedtls_platform_zeroize(&word, sizeof(word));
  if (ret == 0)
  {
    *pOutLen = Len;
  }
  return ret;
}

/* Mutexes */

static void mutex_init(mbedtls_threading_mutex_t *pMutex)
{
  if (pMutex != NULL)
  {
    pMutex->Handle = xSemaphoreCreateMutexStatic(&pMutex->Storage);
  }
}

static void mutex_free(mbedtls_threading_mutex_t *pMutex)
{
  if ((pMutex != NULL) && (pMutex->Handle != NULL))
  {
    vSemaphoreDelete(pMutex->Handle);
    pMutex->Handle = NULL;
  }
}

static int mutex_lock(mbedtls_threading_mutex_t *pMutex)
{
  if ((pMutex == NULL) || (pMutex->Handle == NULL))
  {
    return MBEDTLS_ERR_THREADING_BAD_INPUT_DATA;
  }
  return (xSemaphoreTake(pMutex->Handle, portMAX_DELAY) == pdTRUE) ? 0
                                                                  : MBEDTLS_ERR_THREADING_MUTEX_ERROR;
}

static int mutex_unlock(mbedtls_threading_mutex_t *pMutex)
{
  if ((pMutex == NULL) || (pMutex->Handle == NULL))
  {
    return MBEDTLS_ERR_THREADING_BAD_INPUT_DATA;
  }
  return (xSemaphoreGive(pMutex->Handle) == pdTRUE) ? 0 : MBEDTLS_ERR_THREADING_MUTEX_ERROR;
}

/* Start */

/* Reads both clocks, which moves them on: see ClockTimer. */
static void clock_fold(TimerHandle_t pTimer)
{
  (void)pTimer;
  (void)N6Tls_GetTime();
  (void)mbedtls_ms_time();
}

int N6Tls_Init(void)
{
  uint32_t discard;

  if (Ready != 0U)
  {
    return 0;
  }

  /* RNG with clock error detection; its first word after start is
   * discarded. */
  RngLock = xSemaphoreCreateMutexStatic(&RngLockStorage);
  __HAL_RCC_RNG_CLK_ENABLE();
  Rng.Instance                    = RNG;
  Rng.State                       = HAL_RNG_STATE_RESET;
  Rng.Init.ClockErrorDetection    = RNG_CED_ENABLE;
  if ((HAL_RNG_Init(&Rng) != HAL_OK) || (rng_word(&discard) != 0))
  {
    Rng.Instance = NULL;
    return N6TLS_ERR_RNG;
  }

  /* Mutexes first: psa_crypto_init() takes them. */
  mbedtls_threading_set_alt(mutex_init, mutex_free, mutex_lock, mutex_unlock);
  if (psa_crypto_init() != PSA_SUCCESS)
  {
    return N6TLS_ERR_CRYPTO_INIT;
  }

  if (ClockTimer == NULL)
  {
    ClockTimer = xTimerCreateStatic("tlsclock", pdMS_TO_TICKS(CLOCK_FOLD_MS), pdTRUE, NULL,
                                    clock_fold, &ClockTimerStorage);
  }
  if ((ClockTimer == NULL) || (xTimerStart(ClockTimer, 0U) != pdPASS))
  {
    return N6TLS_ERR_NOT_READY;
  }
  Ready = 1U;
  return 0;
}

/* Time */

void N6Tls_SetTime(uint32_t UnixSeconds)
{
  taskENTER_CRITICAL();
  TimeBase = UnixSeconds;
  TimeTick = HAL_GetTick();
  taskEXIT_CRITICAL();
}

uint32_t N6Tls_GetTime(void)
{
  uint32_t now;
  uint32_t seconds;

  /* Fold whole seconds into the base, so a tick wrap loses nothing. */
  taskENTER_CRITICAL();
  if (TimeBase != 0U)
  {
    now      = HAL_GetTick();
    seconds  = (now - TimeTick) / 1000U;
    TimeBase += seconds;
    TimeTick += seconds * 1000U;
  }
  now = TimeBase;
  taskEXIT_CRITICAL();
  return now;
}

time_t N6Tls_MbedtlsTime(time_t *pTime)
{
  const time_t now = (time_t)N6Tls_GetTime();

  if (pTime != NULL)
  {
    *pTime = now;
  }
  return now;
}

mbedtls_ms_time_t mbedtls_ms_time(void)
{
  uint32_t now;
  uint64_t ms;

  taskENTER_CRITICAL();
  now = HAL_GetTick();
  if (now < MsLast)
  {
    MsHigh++;
  }
  MsLast = now;
  ms     = ((uint64_t)MsHigh << 32) | now;
  taskEXIT_CRITICAL();
  return (mbedtls_ms_time_t)ms;
}

/* Identity */

static int dns_name_ok(const char *pName)
{
  size_t n = 0U;

  if (pName == NULL)
  {
    return 0;
  }
  for (; pName[n] != '\0'; n++)
  {
    const char c = pName[n];

    if (!(((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
          ((c >= '0') && (c <= '9')) || (c == '-') || (c == '.')))
    {
      return 0;
    }
  }
  return (n > 0U) && (n <= 63U);
}

/* Mbed TLS writes DER backwards from the end of the buffer. */
static void to_front(uint8_t *pBuf, size_t Size, size_t Len)
{
  memmove(pBuf, &pBuf[Size - Len], Len);
  memset(&pBuf[Len], 0, Size - Len);
}

int N6Tls_MakeIdentity(N6Tls_Identity *pId, const char *pDnsName, uint32_t Ipv4)
{
  static const char        valid_from[] = "20260101000000";
  static const char        valid_to[]   = "20491231235959";
  mbedtls_pk_context       key;
  mbedtls_x509write_cert   crt;
  mbedtls_x509_san_list    san[2];
  mbedtls_asn1_sequence    eku;
  uint8_t                  ip[4];
  uint8_t                  serial[16];
  char                     subject[96];
  int                      ret;

  if ((pId == NULL) || !dns_name_ok(pDnsName))
  {
    return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
  }
  if (Ready == 0U)
  {
    return N6TLS_ERR_NOT_READY;
  }
  memset(pId, 0, sizeof(*pId));
  mbedtls_pk_init(&key);
  mbedtls_x509write_crt_init(&crt);

  ret = mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
  if (ret == 0)
  {
    ret = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
                              mbedtls_psa_get_random, MBEDTLS_PSA_RANDOM_STATE);
  }

  /* Random positive serial (RFC 5280); the key signs its own certificate. */
  if (ret == 0)
  {
    ret = (psa_generate_random(serial, sizeof(serial)) == PSA_SUCCESS) ? 0 : N6TLS_ERR_RNG;
    serial[0] = (uint8_t)((serial[0] & 0x7FU) | 0x40U);
  }
  if (ret == 0)
  {
    (void)snprintf(subject, sizeof(subject), "O=Neuro N6,CN=%s", pDnsName);
    mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_subject_key(&crt, &key);
    mbedtls_x509write_crt_set_issuer_key(&crt, &key);
    ret = mbedtls_x509write_crt_set_serial_raw(&crt, serial, sizeof(serial));
  }
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_set_subject_name(&crt, subject);
  }
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_set_issuer_name(&crt, subject);
  }
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_set_validity(&crt, valid_from, valid_to);
  }

  /* TLS server end entity: not a CA, signs handshakes, answers to its name
   * and, if given, its address. */
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_set_basic_constraints(&crt, 0, -1);
  }
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_set_key_usage(&crt, MBEDTLS_X509_KU_DIGITAL_SIGNATURE);
  }
  if (ret == 0)
  {
    memset(&eku, 0, sizeof(eku));
    eku.buf.tag = MBEDTLS_ASN1_OID;
    eku.buf.p   = (unsigned char *)MBEDTLS_OID_SERVER_AUTH;
    eku.buf.len = MBEDTLS_OID_SIZE(MBEDTLS_OID_SERVER_AUTH);
    ret = mbedtls_x509write_crt_set_ext_key_usage(&crt, &eku);
  }
  if (ret == 0)
  {
    memset(san, 0, sizeof(san));
    san[0].node.type                         = MBEDTLS_X509_SAN_DNS_NAME;
    san[0].node.san.unstructured_name.p      = (unsigned char *)pDnsName;
    san[0].node.san.unstructured_name.len    = strlen(pDnsName);
    if (Ipv4 != 0U)
    {
      ip[0] = (uint8_t)Ipv4;
      ip[1] = (uint8_t)(Ipv4 >> 8);
      ip[2] = (uint8_t)(Ipv4 >> 16);
      ip[3] = (uint8_t)(Ipv4 >> 24);
      san[0].next                            = &san[1];
      san[1].node.type                       = MBEDTLS_X509_SAN_IP_ADDRESS;
      san[1].node.san.unstructured_name.p    = ip;
      san[1].node.san.unstructured_name.len  = sizeof(ip);
    }
    ret = mbedtls_x509write_crt_set_subject_alternative_name(&crt, san);
  }

  /* Both in DER, and the certificate's fingerprint. */
  if (ret == 0)
  {
    ret = mbedtls_x509write_crt_der(&crt, pId->Cert, sizeof(pId->Cert),
                                    mbedtls_psa_get_random, MBEDTLS_PSA_RANDOM_STATE);
    if (ret > 0)
    {
      pId->CertLen = (size_t)ret;
      to_front(pId->Cert, sizeof(pId->Cert), pId->CertLen);
      ret = 0;
    }
  }
  if (ret == 0)
  {
    ret = mbedtls_pk_write_key_der(&key, pId->Key, sizeof(pId->Key));
    if (ret > 0)
    {
      pId->KeyLen = (size_t)ret;
      to_front(pId->Key, sizeof(pId->Key), pId->KeyLen);
      ret = 0;
    }
  }
  if (ret == 0)
  {
    ret = mbedtls_sha256(pId->Cert, pId->CertLen, pId->Sha256, 0);
  }

  mbedtls_x509write_crt_free(&crt);
  mbedtls_pk_free(&key);
  if (ret != 0)
  {
    mbedtls_platform_zeroize(pId, sizeof(*pId));
  }
  return ret;
}

int N6Tls_CertPem(const N6Tls_Identity *pId, char *pBuf, size_t Size)
{
  size_t len = 0U;

  if ((pId == NULL) || (pId->CertLen == 0U) || (pBuf == NULL))
  {
    return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
  }
  return mbedtls_pem_write_buffer("-----BEGIN CERTIFICATE-----\n", "-----END CERTIFICATE-----\n",
                                  pId->Cert, pId->CertLen, (unsigned char *)pBuf, Size, &len);
}

void N6Tls_Fingerprint(const N6Tls_Identity *pId, char *pBuf, size_t Size)
{
  size_t at = 0U;

  if ((pBuf == NULL) || (Size == 0U))
  {
    return;
  }
  pBuf[0] = '\0';
  for (size_t i = 0U; (pId != NULL) && (i < sizeof(pId->Sha256)) && ((at + 4U) <= Size); i++)
  {
    at += (size_t)snprintf(&pBuf[at], Size - at, (i == 0U) ? "%02X" : ":%02X", pId->Sha256[i]);
  }
}

void N6Tls_WipeKey(N6Tls_Identity *pId)
{
  if (pId != NULL)
  {
    mbedtls_platform_zeroize(pId->Key, sizeof(pId->Key));
    pId->KeyLen = 0U;
  }
}

/* Client */

/* A connected, blocking socket to pHost:Port, or a negative code. Connects
 * non-blocking so that TimeoutMs bounds it. */
static int open_socket(const char *pHost, uint16_t Port, uint32_t TimeoutMs, int *pSocket)
{
  struct addrinfo    hints;
  struct addrinfo   *res = NULL;
  struct sockaddr_in to;
  struct timeval     tv;
  fd_set             wr;
  int                err  = 0;
  socklen_t          elen = sizeof(err);
  int                on   = 1;
  int                off  = 0;
  int                s;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if ((lwip_getaddrinfo(pHost, NULL, &hints, &res) != 0) || (res == NULL))
  {
    return MBEDTLS_ERR_NET_UNKNOWN_HOST;
  }
  memcpy(&to, res->ai_addr, sizeof(to));
  lwip_freeaddrinfo(res);
  to.sin_port = lwip_htons(Port);

  s = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0)
  {
    return MBEDTLS_ERR_NET_SOCKET_FAILED;
  }
  (void)lwip_ioctl(s, FIONBIO, &on);
  (void)lwip_connect(s, (struct sockaddr *)&to, sizeof(to));

  /* Writable once connected or failed; SO_ERROR says which. */
  FD_ZERO(&wr);
  FD_SET(s, &wr);
  tv.tv_sec  = (long)(TimeoutMs / 1000U);
  tv.tv_usec = (long)((TimeoutMs % 1000U) * 1000U);
  if ((lwip_select(s + 1, NULL, &wr, NULL, &tv) != 1) ||
      (lwip_getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &elen) != 0) || (err != 0))
  {
    (void)lwip_close(s);
    return MBEDTLS_ERR_NET_CONNECT_FAILED;
  }
  (void)lwip_ioctl(s, FIONBIO, &off);

  /* Writes wait at most TimeoutMs; reads use Mbed TLS's read timeout, which
   * NXP's transport applies. */
  (void)lwip_setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  *pSocket = s;
  return 0;
}

/* NXP's transport reports a timed-out read or write as WANT_READ/WANT_WRITE:
 * it takes any non-zero F_GETFL, which lwIP 2.2 always returns, as
 * non-blocking. Retry until TimeoutMs has passed since the operation began. */
static int want_more(int Ret, uint32_t Start, uint32_t TimeoutMs)
{
  return ((Ret == MBEDTLS_ERR_SSL_WANT_READ) || (Ret == MBEDTLS_ERR_SSL_WANT_WRITE)) &&
         ((HAL_GetTick() - Start) < TimeoutMs);
}

static int timed_out(int Ret)
{
  return (Ret == MBEDTLS_ERR_SSL_WANT_READ) || (Ret == MBEDTLS_ERR_SSL_WANT_WRITE);
}

int N6Tls_Connect(N6Tls_Client *pClient, const char *pHost, uint16_t Port,
                  mbedtls_x509_crt *pTrust, uint32_t TimeoutMs)
{
  uint32_t start;
  int      ret;

  if (pClient == NULL)
  {
    return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
  }
  memset(pClient, 0, sizeof(*pClient));
  pClient->Socket    = -1;
  pClient->TimeoutMs = (TimeoutMs == 0U) ? 1U : TimeoutMs;
  mbedtls_ssl_config_init(&pClient->Config);
  mbedtls_ssl_init(&pClient->Ssl);

  if ((pHost == NULL) || (pTrust == NULL))
  {
    return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
  }
  if (Ready == 0U)
  {
    return N6TLS_ERR_NOT_READY;
  }
  /* No time, no date check: refuse rather than trust a certificate that may
   * have expired. */
  if (N6Tls_GetTime() == 0U)
  {
    return N6TLS_ERR_NO_CLOCK;
  }

  /* Require a valid chain, in date and naming pHost; reads time out. */
  ret = mbedtls_ssl_config_defaults(&pClient->Config, MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
  if (ret == 0)
  {
    mbedtls_ssl_conf_authmode(&pClient->Config, MBEDTLS_SSL_VERIFY_REQUIRED);
    mbedtls_ssl_conf_ca_chain(&pClient->Config, pTrust, NULL);
    mbedtls_ssl_conf_rng(&pClient->Config, mbedtls_psa_get_random, MBEDTLS_PSA_RANDOM_STATE);
    mbedtls_ssl_conf_read_timeout(&pClient->Config, pClient->TimeoutMs);
    ret = mbedtls_ssl_setup(&pClient->Ssl, &pClient->Config);
  }
  if (ret == 0)
  {
    ret = mbedtls_ssl_set_hostname(&pClient->Ssl, pHost);
  }
  if (ret == 0)
  {
    ret = open_socket(pHost, Port, pClient->TimeoutMs, &pClient->Socket);
  }
  if (ret == 0)
  {
    wm_mbedtls_set_fd(&pClient->Ssl, pClient->Socket);
    start = HAL_GetTick();
    do
    {
      ret = mbedtls_ssl_handshake(&pClient->Ssl);
    } while (want_more(ret, start, pClient->TimeoutMs));
    if (timed_out(ret))
    {
      ret = MBEDTLS_ERR_SSL_TIMEOUT;
    }
  }

  if (ret != 0)
  {
    pClient->VerifyFlags = mbedtls_ssl_get_verify_result(&pClient->Ssl);
    if (pClient->VerifyFlags == (uint32_t)-1)
    {
      pClient->VerifyFlags = 0U;
    }
    N6Tls_Close(pClient);
    return ret;
  }
  pClient->Open = 1U;
  return 0;
}

int N6Tls_Write(N6Tls_Client *pClient, const void *pData, size_t Len)
{
  const unsigned char *p    = (const unsigned char *)pData;
  size_t               done = 0U;

  if ((pClient == NULL) || (pClient->Open == 0U))
  {
    return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
  }
  while (done < Len)
  {
    const uint32_t start = HAL_GetTick();
    int            ret;

    do
    {
      ret = mbedtls_ssl_write(&pClient->Ssl, &p[done], Len - done);
    } while (want_more(ret, start, pClient->TimeoutMs));
    if (ret <= 0)
    {
      return timed_out(ret) ? MBEDTLS_ERR_SSL_TIMEOUT : ((ret == 0) ? MBEDTLS_ERR_SSL_INTERNAL_ERROR : ret);
    }
    done += (size_t)ret;
  }
  return (int)Len;
}

int N6Tls_Read(N6Tls_Client *pClient, void *pBuf, size_t Size)
{
  const uint32_t start = HAL_GetTick();
  int            ret;

  if ((pClient == NULL) || (pClient->Open == 0U))
  {
    return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
  }
  do
  {
    ret = mbedtls_ssl_read(&pClient->Ssl, (unsigned char *)pBuf, Size);
  } while (want_more(ret, start, pClient->TimeoutMs));

  if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
  {
    return 0;
  }
  if (ret == 0)
  {
    return MBEDTLS_ERR_SSL_CONN_EOF;
  }
  return timed_out(ret) ? MBEDTLS_ERR_SSL_TIMEOUT : ret;
}

void N6Tls_Close(N6Tls_Client *pClient)
{
  if (pClient == NULL)
  {
    return;
  }
  if ((pClient->Open != 0U) && (pClient->Socket >= 0))
  {
    (void)mbedtls_ssl_close_notify(&pClient->Ssl);
  }
  pClient->Open = 0U;
  if (pClient->Socket >= 0)
  {
    (void)lwip_close(pClient->Socket);
    pClient->Socket = -1;
  }
  mbedtls_ssl_free(&pClient->Ssl);
  mbedtls_ssl_config_free(&pClient->Config);
}

/* Diagnostics */

const char *N6Tls_Strerror(int Err, char *pBuf, size_t Size)
{
  const char *own = NULL;

  if ((pBuf == NULL) || (Size == 0U))
  {
    return "";
  }
  switch (Err)
  {
    case N6TLS_ERR_NOT_READY:             own = "N6Tls_Init() has not succeeded"; break;
    case N6TLS_ERR_NO_CLOCK:              own = "the time is not set, so certificates cannot be checked"; break;
    case N6TLS_ERR_RNG:                   own = "the random number generator failed"; break;
    case N6TLS_ERR_CRYPTO_INIT:           own = "psa_crypto_init() failed"; break;
    case MBEDTLS_ERR_NET_UNKNOWN_HOST:    own = "NET - the host name did not resolve"; break;
    case MBEDTLS_ERR_NET_SOCKET_FAILED:   own = "NET - no socket could be opened"; break;
    case MBEDTLS_ERR_NET_CONNECT_FAILED:  own = "NET - the connection failed or timed out"; break;
    case MBEDTLS_ERR_NET_RECV_FAILED:     own = "NET - reading from the connection failed"; break;
    case MBEDTLS_ERR_NET_SEND_FAILED:     own = "NET - writing to the connection failed"; break;
    case MBEDTLS_ERR_NET_CONN_RESET:      own = "NET - the connection was reset"; break;
    default:                              break;
  }
  if (own != NULL)
  {
    (void)snprintf(pBuf, Size, "%s", own);
  }
  else
  {
    mbedtls_strerror(Err, pBuf, Size);
  }
  return pBuf;
}
