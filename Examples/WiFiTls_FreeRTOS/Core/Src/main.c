/*
  WiFiTls_FreeRTOS

  TLS over Wi-Fi: an HTTPS server on the board (port 443: a page, /status.cgi,
  /led.cgi and /fetch.cgi), and an HTTPS client that fetches
  https://valid-isrgrootx1.letsencrypt.org/ once the board has network time.
  The network is the one stored on the board, else the access point
  "NeuroN6-TLS" (WPA3 with WPA2 beside it) with the board's own password. Each
  start makes a new ECDSA P-256 key, kept in RAM, and a self-signed
  certificate: accept it only if its SHA-256 fingerprint matches the
  console's. README.md has the details.

  The circuit:
  - Uses the MAYA-W476 radio module, the RGB LED and the USER button. No
    wiring needed.
  - The radio's firmware must already be in external flash at 0x91E00000.

  Send 'b' on the USB console at any time to return to the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_wdg.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "tusb.h"

#include "MayaW476.h"
#include "NeuroN6_FrameQueue.h"
#include "n6_host_contract.h"
#include "n6_asset.h"
#include "n6_asset_host.h"

#include "httpsrv.h"
#include "n6_tls.h"

#include "mbedtls/version.h"
#include "mbedtls/x509_crt.h"

#include <malloc.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

/* Wi-Fi */

/* SoftAP hosts WIFI_AP_SSID; station mode joins WIFI_STA_SSID. The values
 * are the host protocol's transport numbers. */
#define WIFI_MODE_SOFTAP          1
#define WIFI_MODE_STA             2
#ifndef WIFI_MODE
#define WIFI_MODE                 WIFI_MODE_SOFTAP
#endif
_Static_assert((WIFI_MODE_SOFTAP == TRANSPORT_WIFI_SOFTAP) && (WIFI_MODE_STA == TRANSPORT_WIFI_STA),
               "WIFI_MODE_* must be the host protocol's transport values");

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID              "NeuroN6-TLS"
#endif
#ifndef WIFI_AP_CHANNEL
#define WIFI_AP_CHANNEL           6U   /* 1..11, or 0 to choose; 2.4 GHz only */
#endif
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID             "YourNetwork"
#endif
#ifndef WIFI_STA_PSK
#define WIFI_STA_PSK              "YourPassword"
#endif

/* Security in both modes: WPA3 with WPA2 beside it, or WPA3 alone
 * (WIFI_SECURITY_WPA3_SAE) or WPA2 alone (WIFI_SECURITY_WPA2). A
 * passphrase is 8 to 63 characters. */
#ifndef WIFI_SECURITY
#define WIFI_SECURITY             WIFI_SECURITY_WPA2_WPA3
#endif

/* 1: Wi-Fi settings a host stored on the board over USB override the ones
 * above. */
#ifndef WIFI_USE_STORED
#define WIFI_USE_STORED           1
#endif

#define WIFI_TIMEOUT_MS           10000U

/* TLS */

/* Concurrent server sessions, each a task and a TLS session. */
#define HTTPS_SESSIONS            4U

/* The client's request, and its timeout for each step. */
#ifndef CLIENT_HOST
#define CLIENT_HOST               "valid-isrgrootx1.letsencrypt.org"
#endif
#ifndef CLIENT_PATH
#define CLIENT_PATH               "/"
#endif
#define CLIENT_TIMEOUT_MS         10000U

/* How often network time is requested again. */
#define TIME_REFRESH_MS           (60U * 60U * 1000U)

/* The roots the client trusts: Let's Encrypt's two. A client that talks to
 * other servers adds the roots those chain to. */
static const char TrustedRoots[] =
  /* ISRG Root X1, SHA-256
     96:BC:EC:06:26:49:76:F3:74:60:77:9A:CF:28:C5:A7:CF:E8:A3:C0:AA:E1:1A:8F:FC:EE:05:C0:BD:DF:08:C6 */
  "-----BEGIN CERTIFICATE-----\n"
  "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\n"
  "-----END CERTIFICATE-----\n"
  /* ISRG Root X2, SHA-256
     69:72:9B:8E:15:A8:6E:FC:17:7A:57:AF:B7:17:1D:FC:64:AD:D2:8C:2F:CA:8C:F1:50:7E:34:45:3C:CB:14:70 */
  "-----BEGIN CERTIFICATE-----\n"
  "MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw\n"
  "CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg\n"
  "R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00\n"
  "MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT\n"
  "ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw\n"
  "EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW\n"
  "+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9\n"
  "ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T\n"
  "AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI\n"
  "zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW\n"
  "tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1\n"
  "/q4AaOeMSQ+2b1tbFfLn\n"
  "-----END CERTIFICATE-----\n";

/* The page, from web/ by NXP's mkfs.pl (httpsrv_fs_data.c). */
extern const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[];

/* Tasks and timing */

#define LOOP_MS                   20U

#define WDG_TIMEOUT_MS            2000U
#define WDG_WARN_MS               300U
#define USB_BOOT_PUMP_MS          500U

#define APP_TASK_STACK_WORDS      ((16U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (4096U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)

#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

static void mpu_config(void);

/* State */

static char WifiLine[96];
static char ServeLine[128];
static char NameLine[128];
static char DnsName[48];
static char Fingerprint[96];
static char CertPem[1100];

static N6Tls_Identity   Identity;
static mbedtls_x509_crt Trust;
static bool             TrustOk;
static bool             Station;
static uint32_t         Server;

/* What the CGIs read and the app task writes, under Lock. */
static SemaphoreHandle_t Lock;
static StaticSemaphore_t LockStorage;
static char              ClientLine[200] = "not run yet";
static volatile uint8_t  Led;
static volatile bool     FetchWanted;

/* Helpers */

static void set_led(uint8_t Rgb)
{
  Led = (uint8_t)(Rgb & 7U);
  (void)(((Led & 1U) != 0U) ? BSP_LED_On(LED_RED)   : BSP_LED_Off(LED_RED));
  (void)(((Led & 2U) != 0U) ? BSP_LED_On(LED_GREEN) : BSP_LED_Off(LED_GREEN));
  (void)(((Led & 4U) != 0U) ? BSP_LED_On(LED_BLUE)  : BSP_LED_Off(LED_BLUE));
}

/* The client's last result, for the console and the page. */
static void set_client_line(const char *pText)
{
  (void)xSemaphoreTake(Lock, portMAX_DELAY);
  (void)snprintf(ClientLine, sizeof(ClientLine), "%s", pText);
  (void)xSemaphoreGive(Lock);
  print("[HTTPS] client: %s\r\n", pText);
}

static const char *security_name(WifiSecurity Sec)
{
  switch (Sec)
  {
    case WIFI_SECURITY_WPA2_WPA3: return "WPA2/WPA3";
    case WIFI_SECURITY_WPA3_SAE:  return "WPA3";
    case WIFI_SECURITY_WPA2:      return "WPA2";
    default:                      return "?";
  }
}

static void utc_string(uint32_t Unix, char *pOut, size_t Size)
{
  const time_t t = (time_t)Unix;
  struct tm    tm;

  (void)gmtime_r(&t, &tm);
  if (strftime(pOut, Size, "%Y-%m-%d %H:%M:%S UTC", &tm) == 0U)
  {
    pOut[0] = '\0';
  }
}

/* PEM ends its lines with '\n' alone; the console wants "\r\n". */
static void print_lines(const char *pText)
{
  while (*pText != '\0')
  {
    const char  *end = strchr(pText, '\n');
    const size_t len = (end != NULL) ? (size_t)(end - pText) : strlen(pText);

    print("%.*s\r\n", (int)len, pText);
    pText += len + ((end != NULL) ? 1U : 0U);
  }
}

/* The client */

/* GET https://pHost/pPath, verified against TrustedRoots, reported to the
 * console and the page. HTTP/1.0: the reply is not chunked and ends when the
 * server closes. */
static void https_get(const char *pHost, const char *pPath)
{
  static char  buf[1024];
  N6Tls_Client client;
  char         line[200];
  char         why[100];
  char         subject[100] = "?";
  const char  *hdr_end;
  uint32_t     start = HAL_GetTick();
  uint32_t     got   = 0U;
  uint32_t     body  = 0U;
  int          status = 0;
  int          ret;
  int          n;

  print("[HTTPS] GET https://%s%s\r\n", pHost, pPath);
  if (!TrustOk)
  {
    set_client_line("no trusted roots");
    return;
  }
  ret = N6Tls_Connect(&client, pHost, 443U, &Trust, CLIENT_TIMEOUT_MS);
  if (ret != 0)
  {
    (void)N6Tls_Strerror(ret, why, sizeof(why));
    print("[HTTPS] refused: %s (-0x%04X)\r\n", why, (unsigned)-ret);
    if (client.VerifyFlags != 0U)
    {
      (void)mbedtls_x509_crt_verify_info(buf, sizeof(buf), "[HTTPS]   ", client.VerifyFlags);
      print_lines(buf);
    }
    (void)snprintf(line, sizeof(line), "https://%s%s refused: %s", pHost, pPath, why);
    set_client_line(line);
    N6Tls_Close(&client);
    return;
  }
  if (mbedtls_ssl_get_peer_cert(&client.Ssl) != NULL)
  {
    (void)mbedtls_x509_dn_gets(subject, sizeof(subject), &mbedtls_ssl_get_peer_cert(&client.Ssl)->subject);
  }
  print("[HTTPS] %s %s, server %s, certificate verified\r\n", mbedtls_ssl_get_version(&client.Ssl),
        mbedtls_ssl_get_ciphersuite(&client.Ssl), subject);

  n   = snprintf(buf, sizeof(buf), "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: NeuroN6-WiFiTls\r\n\r\n",
                 pPath, pHost);
  ret = N6Tls_Write(&client, buf, (size_t)n);

  while (ret >= 0)
  {
    ret = N6Tls_Read(&client, &buf[got], sizeof(buf) - 1U - got);
    if (ret <= 0)
    {
      break;
    }
    if (status == 0)
    {
      got += (uint32_t)ret;
      buf[got] = '\0';
      hdr_end  = strstr(buf, "\r\n\r\n");
      if (hdr_end == NULL)
      {
        if (got >= (sizeof(buf) - 1U))
        {
          ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;   /* headers too long for this example */
        }
        continue;
      }
      if (sscanf(buf, "HTTP/%*d.%*d %d", &status) != 1)
      {
        status = -1;
      }
      body = got - (uint32_t)((hdr_end + 4) - buf);
      got  = 0U;
    }
    else
    {
      body += (uint32_t)ret;
    }
  }

  if (status > 0)
  {
    /* Without a length, a close with no close_notify cannot be told from a
     * cut connection, so the line says which it was. */
    (void)snprintf(line, sizeof(line), "https://%s%s: HTTP %d, %lu bytes, %s %s, %lu ms%s", pHost, pPath,
                   status, (unsigned long)body, mbedtls_ssl_get_version(&client.Ssl),
                   mbedtls_ssl_get_ciphersuite(&client.Ssl), (unsigned long)(HAL_GetTick() - start),
                   (ret == MBEDTLS_ERR_SSL_CONN_EOF) ? ", closed without close_notify" : "");
  }
  else if (status < 0)
  {
    (void)snprintf(line, sizeof(line), "https://%s%s: the reply is not HTTP", pHost, pPath);
  }
  else
  {
    (void)N6Tls_Strerror((ret < 0) ? ret : MBEDTLS_ERR_SSL_CONN_EOF, why, sizeof(why));
    (void)snprintf(line, sizeof(line), "https://%s%s: no complete reply (%s)", pHost, pPath, why);
  }
  set_client_line(line);
  N6Tls_Close(&client);
}

/* The server */

/* Quote and escape for JSON: the SSID and the client's report are untrusted. */
static size_t json_str(char *pOut, size_t Size, const char *pIn)
{
  size_t at = 0U;

  if (Size < 3U)
  {
    return 0U;
  }
  pOut[at++] = '"';
  for (; (*pIn != '\0') && ((at + 7U) < Size); pIn++)
  {
    const unsigned char c = (unsigned char)*pIn;

    if ((c == '"') || (c == '\\'))
    {
      pOut[at++] = '\\';
      pOut[at++] = (char)c;
    }
    else if (c < 0x20U)
    {
      at += (size_t)snprintf(&pOut[at], Size - at, "\\u%04x", c);
    }
    else
    {
      pOut[at++] = (char)c;
    }
  }
  pOut[at++] = '"';
  pOut[at]   = '\0';
  return at;
}

static int status_json(char *pOut, size_t Size)
{
  const struct mallinfo mi = mallinfo();
  const uint32_t        ip = MayaW476_LocalIP();
  char                  name[64], ssid[80], client[240];
  char                  version[40];

  (void)json_str(name, sizeof(name), DnsName);
  (void)json_str(ssid, sizeof(ssid), MayaW476_SSID());
  (void)xSemaphoreTake(Lock, portMAX_DELAY);
  (void)json_str(client, sizeof(client), ClientLine);
  (void)xSemaphoreGive(Lock);
  mbedtls_version_get_string_full(version);

  return snprintf(pOut, Size,
                  "{\"name\":%s,\"ip\":\"%u.%u.%u.%u\",\"ssid\":%s,\"rssi\":%ld,\"uptime\":%lu,"
                  "\"button\":%d,\"led\":%u,\"heap\":%lu,\"kernel\":%lu,"
                  "\"fingerprint\":\"%s\",\"mbedtls\":\"%s\",\"time\":%lu,\"client\":%s}",
                  name, (unsigned)(ip & 0xFFU), (unsigned)((ip >> 8) & 0xFFU),
                  (unsigned)((ip >> 16) & 0xFFU), (unsigned)(ip >> 24), ssid,
                  (long)(Station ? MayaW476_RSSI() : 0),
                  (unsigned long)(HAL_GetTick() / 1000U),
                  (BSP_PB_GetState(BUTTON_USER) == (int32_t)BUTTON_PRESSED) ? 1 : 0, (unsigned)Led,
                  (unsigned long)mi.uordblks, (unsigned long)xPortGetFreeHeapSize(), Fingerprint,
                  version, (unsigned long)N6Tls_GetTime(), client);
}

static void cgi_reply(uint32_t Session, uint32_t Code, const char *pBody, int32_t Len, bool Head)
{
  HTTPSRV_CGI_RES_STRUCT res;

  memset(&res, 0, sizeof(res));
  res.ses_handle     = Session;
  res.status_code    = Code;
  res.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
  res.content_length = Len;
  res.data           = (char *)pBody;
  res.data_length    = Head ? 0U : (uint32_t)Len;
  (void)HTTPSRV_cgi_write(&res);
}

/* Uses the session task's stack: sessions run CGIs concurrently. */
static void reply_status(uint32_t Session, bool Head)
{
  char json[640];
  int  n = status_json(json, sizeof(json));

  if ((n < 0) || ((size_t)n >= sizeof(json)))
  {
    cgi_reply(Session, HTTPSRV_CODE_INTERNAL_ERROR, NULL, 0, false);
    return;
  }
  cgi_reply(Session, HTTPSRV_CODE_OK, json, n, Head);
}

/* The form body of a POST, NUL-terminated; false if it is too long. */
static bool read_body(const HTTPSRV_CGI_REQ_STRUCT *pReq, char *pOut, size_t Size)
{
  uint32_t got = 0U;

  if (pReq->content_length >= Size)
  {
    return false;
  }
  while (got < pReq->content_length)
  {
    const uint32_t n = HTTPSRV_cgi_read(pReq->ses_handle, &pOut[got], pReq->content_length - got);

    if ((n == 0U) || (n > (pReq->content_length - got)))
    {
      break;
    }
    got += n;
  }
  pOut[got] = '\0';
  return got == pReq->content_length;
}

static int status_cgi(HTTPSRV_CGI_REQ_STRUCT *pReq)
{
  if ((pReq->request_method != HTTPSRV_REQ_GET) && (pReq->request_method != HTTPSRV_REQ_HEAD))
  {
    cgi_reply(pReq->ses_handle, HTTPSRV_CODE_METHOD_NOT_ALLOWED, NULL, 0, false);
    return 0;
  }
  reply_status(pReq->ses_handle, pReq->request_method == HTTPSRV_REQ_HEAD);
  return 0;
}

static int led_cgi(HTTPSRV_CGI_REQ_STRUCT *pReq)
{
  char     body[32];
  unsigned rgb;

  if (pReq->request_method != HTTPSRV_REQ_POST)
  {
    cgi_reply(pReq->ses_handle, HTTPSRV_CODE_METHOD_NOT_ALLOWED, NULL, 0, false);
    return 0;
  }
  if (!read_body(pReq, body, sizeof(body)) || (sscanf(body, "rgb=%u", &rgb) != 1) || (rgb > 7U))
  {
    cgi_reply(pReq->ses_handle, HTTPSRV_CODE_BAD_REQ, NULL, 0, false);
    return 0;
  }
  set_led((uint8_t)rgb);
  print("[HTTPS] LED %u from %s\r\n", rgb, pReq->remote_addr);
  reply_status(pReq->ses_handle, false);
  return 0;
}

static int fetch_cgi(HTTPSRV_CGI_REQ_STRUCT *pReq)
{
  char body[32];

  if (pReq->request_method != HTTPSRV_REQ_POST)
  {
    cgi_reply(pReq->ses_handle, HTTPSRV_CODE_METHOD_NOT_ALLOWED, NULL, 0, false);
    return 0;
  }
  (void)read_body(pReq, body, sizeof(body));
  /* Line first: the app task may run the fetch as soon as FetchWanted is set. */
  set_client_line("running...");
  FetchWanted = true;
  reply_status(pReq->ses_handle, false);
  return 0;
}

static const HTTPSRV_CGI_LINK_STRUCT Cgis[] =
{
  { "status", status_cgi },
  { "led",    led_cgi    },
  { "fetch",  fetch_cgi  },
  { NULL,     NULL       }
};

static void start_server(void)
{
  static HTTPSRV_TLS_PARAM_STRUCT tls;
  static HTTPSRV_PARAM_STRUCT     params;

  HTTPSRV_FS_init(httpsrv_fs_data);

  memset(&params, 0, sizeof(params));
  params.address.ss_family = AF_INET;         /* port 0: HTTPS's 443 */
  params.root_dir          = "";
  params.index_page        = "/index.html";
  params.max_ses           = HTTPS_SESSIONS;
  params.cgi_lnk_tbl       = Cgis;
  params.tls_param         = &tls;
  tls.certificate_buffer      = Identity.Cert;
  tls.certificate_buffer_size = Identity.CertLen;
  tls.private_key_buffer      = Identity.Key;
  tls.private_key_buffer_size = Identity.KeyLen;

  Server = HTTPSRV_init(&params);

  /* The server has parsed its own copy of the key. */
  N6Tls_WipeKey(&Identity);
  if (Server == 0U)
  {
    print("[HTTPS] the server did not start\r\n");
  }
}

/* The application */

static void print_identity(void)
{
  print("%s\r\n", NameLine);
  print("[TLS] certificate SHA-256 %s\r\n", Fingerprint);
  print_lines(CertPem);
}

static void print_summary(void)
{
  if (WifiLine[0] != '\0')
  {
    print("%s\r\n", WifiLine);
  }
  if (Fingerprint[0] != '\0')
  {
    print_identity();
  }
  if (ServeLine[0] != '\0')
  {
    print("%s\r\n", ServeLine);
    (void)xSemaphoreTake(Lock, portMAX_DELAY);
    print("[HTTPS] client: %s\r\n", ClientLine);
    (void)xSemaphoreGive(Lock);
  }
}

static void sync_time(void)
{
  const uint32_t now = Station ? MayaW476_GetTime() : 0U;
  char           when[32];

  if (now != 0U)
  {
    N6Tls_SetTime(now);
    utc_string(now, when, sizeof(when));
    print("[TLS] time %s, from the network\r\n", when);
  }
}

static void setup(void)
{
  char        version[40];
  char        why[100];
  wl_status_t wifi_status;
  int         ret;

  print("=== WiFiTls: an HTTPS server and client over Wi-Fi, 'b' for the bootloader ===\r\n");
  Lock = xSemaphoreCreateMutexStatic(&LockStorage);
  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);
  (void)BSP_LED_Init(LED_BLUE);
  set_led(0U);
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* RNG, Mbed TLS mutexes and PSA, then the client's trusted roots. */
  ret = N6Tls_Init();
  if (ret != 0)
  {
    print("[TLS] %s\r\n", N6Tls_Strerror(ret, why, sizeof(why)));
    return;
  }
  mbedtls_version_get_string_full(version);
  print("[TLS] %s\r\n", version);
  mbedtls_x509_crt_init(&Trust);
  TrustOk = (mbedtls_x509_crt_parse(&Trust, (const unsigned char *)TrustedRoots, sizeof(TrustedRoots)) == 0);

  MayaW476_SetTimeout(WIFI_TIMEOUT_MS);
#if (WIFI_USE_STORED == 1)
  if (MayaW476_StoredMode() != WL_STORED_NONE)
  {
    Station     = (MayaW476_StoredMode() == WL_STORED_STA);
    wifi_status = MayaW476_BeginStored();
    if (Station)
    {
      (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] the network stored on this board, \"%s\"",
                     MayaW476_StoredSSID());
    }
    else
    {
      (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] the access point stored on this board, "
                     "\"%s\"  password: %s", MayaW476_StoredSSID(), MayaW476_SoftApPassword());
    }
    print("%s\r\n", WifiLine);
  }
  else
#endif
  {
#if (WIFI_MODE == WIFI_MODE_SOFTAP)
    /* The board's own password: made and stored on first use, which
     * restarts the board once. */
    (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] access point \"%s\" (%s)  password: %s",
                   WIFI_AP_SSID, security_name(WIFI_SECURITY), MayaW476_DevicePassword());
    print("%s\r\n", WifiLine);
    wifi_status = MayaW476_BeginAP(WIFI_AP_SSID, NULL, WIFI_AP_CHANNEL, WIFI_SECURITY);
#else
    Station = true;
    (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] joining \"%s\" (%s)", WIFI_STA_SSID,
                   security_name(WIFI_SECURITY));
    print("%s\r\n", WifiLine);
    wifi_status = MayaW476_Begin(WIFI_STA_SSID, WIFI_STA_PSK, WIFI_SECURITY);
#endif
  }
  for (int i = 0; (i < 100) && (MayaW476_LocalIP() == 0U); i++)
  {
    vTaskDelay(pdMS_TO_TICKS(100U));
  }
  print("[WiFi] status %d, address %s\r\n", (int)wifi_status, MayaW476_Ipv4String());
  if (MayaW476_LocalIP() == 0U)
  {
    print("[WiFi] no address: no server\r\n");
    return;
  }

  /* Only a station on a network with internet access gets the time. */
  sync_time();

  /* The certificate names the board's mDNS name and address. */
  (void)snprintf(DnsName, sizeof(DnsName), "%s.local", MayaW476_GetHostname());
  ret = N6Tls_MakeIdentity(&Identity, DnsName, MayaW476_LocalIP());
  if (ret != 0)
  {
    print("[TLS] no key: %s\r\n", N6Tls_Strerror(ret, why, sizeof(why)));
    return;
  }
  N6Tls_Fingerprint(&Identity, Fingerprint, sizeof(Fingerprint));
  (void)N6Tls_CertPem(&Identity, CertPem, sizeof(CertPem));
  (void)snprintf(NameLine, sizeof(NameLine), "[TLS] new key, ECDSA P-256, for %s and %s", DnsName,
                 MayaW476_Ipv4String());
  print_identity();

  start_server();
  if (Server != 0U)
  {
    /* The address, not the name: mDNS answers only while the board streams
     * to Neuro Studio, which this example does not. */
    (void)snprintf(ServeLine, sizeof(ServeLine), "[HTTPS] serving https://%s/",
                   MayaW476_Ipv4String());
    print("%s\r\n", ServeLine);
  }

  if (N6Tls_GetTime() != 0U)
  {
    https_get(CLIENT_HOST, CLIENT_PATH);
  }
  else
  {
    set_client_line("not run: the time is not known, so certificates cannot be checked");
  }
}

static void loop(void)
{
  static bool     console_was_up;
  static uint8_t  was_pressed;
  static uint32_t last_sync;
  const bool      console_up = (BSP_COM_IsConnected() != 0);
  const uint8_t   pressed    = (BSP_PB_GetState(BUTTON_USER) == (int32_t)BUTTON_PRESSED) ? 1U : 0U;

  vTaskDelay(pdMS_TO_TICKS(LOOP_MS));

  /* Repeat the key lines whenever a console connects. */
  if (console_up && !console_was_up)
  {
    print_summary();
  }
  console_was_up = console_up;

  if ((pressed != 0U) && (was_pressed == 0U))
  {
    FetchWanted = true;
  }
  was_pressed = pressed;

  if (Station && ((HAL_GetTick() - last_sync) > TIME_REFRESH_MS))
  {
    last_sync = HAL_GetTick();
    sync_time();
  }

  if (FetchWanted)
  {
    FetchWanted = false;
    if (N6Tls_GetTime() == 0U)
    {
      sync_time();
    }
    if (N6Tls_GetTime() != 0U)
    {
      https_get(CLIENT_HOST, CLIENT_PATH);
    }
    else
    {
      set_client_line("not run: the time is not known, so certificates cannot be checked");
    }
  }
}

static void app_task(void *pArg)
{
  (void)pArg;

  setup();
  for (;;)
  {
    loop();
  }
}

/* The Neuro host */

/* The Wi-Fi library hands a connected Neuro host over here. With no stream
 * to give it, this holds the connection, drops what the host sends, and
 * lets go when the host leaves so the library listens again. */
static const n6_net_link_t *volatile NetPending;
static const n6_net_link_t          *Net;

bool NeuroN6_NetAttach(const n6_net_link_t *pLink)
{
  bool free_slot;

  taskENTER_CRITICAL();
  free_slot = (pLink != NULL) && (Net == NULL) && (NetPending == NULL);
  if (free_slot)
  {
    NetPending = pLink;
  }
  taskEXIT_CRITICAL();
  return free_slot;
}

static void net_service(void)
{
  uint8_t in[N6_HOST_CMD_MAX];

  if (Net == NULL)
  {
    taskENTER_CRITICAL();
    Net        = NetPending;
    NetPending = NULL;
    taskEXIT_CRITICAL();
    return;
  }
  /* In parentheses, or lwIP's BSD recv() macro takes the member for itself. */
  if ((Net->recv)(in, sizeof(in)) < 0)
  {
    const n6_net_link_t *link = Net;

    Net = NULL;
    link->closed();
  }
}

/* The discovery reply: one frame with META_BOARD_IPV4 (address and port).
 * The library adds the board's name. */
uint32_t NeuroN6_IpAnnounceFrame(uint8_t *pOut, uint32_t Cap)
{
  const uint32_t       ip   = MayaW476_GetIPv4_BE();
  const uint16_t       port = MayaW476_GetIPv4_Port();
  const n6_frame_hdr_t hdr  = { N6_FRAME_MAGIC, 0U, 11U, 0U, 0U };

  if ((pOut == NULL) || (Cap < (sizeof(hdr) + 11U)) || (ip == 0U))
  {
    return 0U;
  }
  memcpy(pOut, &hdr, sizeof(hdr));
  pOut[20] = 0x20U;   /* META_BOARD_IPV4, 8 bytes: the address, then the port as a u32 */
  pOut[21] = 8U;
  pOut[22] = 0U;
  memcpy(&pOut[23], &ip, 4U);
  pOut[27] = (uint8_t)(port & 0xFFU);
  pOut[28] = (uint8_t)(port >> 8);
  pOut[29] = 0U;
  pOut[30] = 0U;
  return sizeof(hdr) + 11U;
}

/* USB */

/**
  * @brief  Asset-store commands on the USB vendor interface, so a host can
  *         store the Wi-Fi settings (the board resets to write them).
  */
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
{
  uint8_t  cmd[N6_HOST_CMD_MAX];
  uint32_t count;

  (void)idx;
  if (buffer == NULL)
  {
    count = tud_vendor_n_read(0, cmd, sizeof(cmd));
    /* One command per transfer: drop the rest, so the next command starts
     * clean and the FIFO keeps room for it. */
    tud_vendor_n_read_flush(0);
  }
  else
  {
    count = (bufsize < sizeof(cmd)) ? bufsize : sizeof(cmd);
    memcpy(cmd, buffer, count);
  }
  if (n6_asset_host_command(cmd, count) == N6_ASSET_HOST_STAGED)
  {
    n6_asset_apply();
  }
}

/**
  * @brief  Keeps the console and the way back to the bootloader alive, and
  *         services the Neuro host.
  */
static void usb_task(void *pArg)
{
  uint8_t ch;

  (void)pArg;

  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    (void)BSP_WDG_Refresh();
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    while (BSP_COM_Read(&ch, 1U) == 1)
    {
      if (ch == (uint8_t)'b')
      {
        BSP_COM_RebootToBootloader();
      }
    }
    net_service();
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
  }
}

/* Scaffolding */

/**
  * @brief  The Wi-Fi library's console: the USB console (dropped with no host).
  */
void NeuroN6_DbgPrint(const char *msg)
{
  print("%s", msg);
}

/**
  * @brief  Runs 300 ms before the watchdog reset, in whatever context is
  *         stuck: reboot into the bootloader instead.
  */
void BSP_WDG_EWI_Callback(void)
{
  BSP_COM_RebootToBootloader();
}

/**
  * @brief  The external memories are not cacheable.
  */
static void mpu_config(void)
{
  MPU_Region_InitTypeDef     default_config = {0};
  MPU_Attributes_InitTypeDef attr_config    = {0};
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  HAL_MPU_Disable();

  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number     = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  default_config.Enable           = MPU_REGION_ENABLE;
  default_config.Number           = MPU_REGION_NUMBER0;
  default_config.BaseAddress      = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress     = __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  default_config.Number       = MPU_REGION_NUMBER1;
  default_config.BaseAddress  = 0x70000000;
  default_config.LimitAddress = 0x71FFFFFF;
  HAL_MPU_ConfigRegion(&default_config);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(primask_bit);
}

int main(void)
{
  uint32_t start;

  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  /* No SystemClock_Config: the boot loader owns the clock tree and both
   * external memories; the BSP reads the frequencies back at run time. */

  /* Watchdog first; a watchdog reset goes to the bootloader, not back into
   * the code that caused it. */
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* USB first, then the boot-time enumeration pump. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  start = HAL_GetTick();
  while (!tud_mounted() && ((HAL_GetTick() - start) < USB_BOOT_PUMP_MS))
  {
    tud_task_ext(0, false);
    (void)BSP_WDG_Refresh();
    HAL_Delay(10U);
  }

  if ((xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(app_task, "app", APP_TASK_STACK_WORDS, NULL,
                   APP_TASK_PRIORITY, NULL) != pdPASS))
  {
    Error_Handler();
  }

  vTaskStartScheduler();

  Error_Handler();
  return 0;
}

/**
  * @brief  Reboot into the bootloader, where the board can be reflashed.
  */
void Error_Handler(void)
{
  BSP_COM_RebootToBootloader();
}
