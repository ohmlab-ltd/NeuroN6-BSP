/**
  ******************************************************************************
  * @file    n6_tls.h
  * @brief   TLS over Wi-Fi: board glue for Mbed TLS, NXP's HTTPS server and
  *          lwIP's sockets, and a TLS client built from them.
  *
  *          - N6Tls_Init(): TRNG entropy, FreeRTOS mutexes, PSA started.
  *          - N6Tls_SetTime(): time for certificate checks; no client
  *            connection is attempted until it is called.
  *          - N6Tls_MakeIdentity(): a new self-signed ECDSA P-256 key and
  *            certificate in RAM, for NXP's HTTPS server.
  *          - N6Tls_Connect() onwards: a client that checks the server's
  *            chain, dates and name, or fails.
  *
  *          Everything else is Mbed TLS's (mbedtls/ssl.h) and NXP's
  *          (httpsrv.h) own API.
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

#ifndef N6_TLS_H
#define N6_TLS_H

#include <stddef.h>
#include <stdint.h>

#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Errors: functions return 0 or a negative code: one of these (outside Mbed
 * TLS's range), Mbed TLS's, or a network code (MBEDTLS_ERR_NET_*, in NXP's
 * wm_mbedtls_net.h). N6Tls_Strerror() names them all. */
#define N6TLS_ERR_NOT_READY     (-0x10001)  /* N6Tls_Init() has not succeeded   */
#define N6TLS_ERR_NO_CLOCK      (-0x10002)  /* no time set: dates can't be checked */
#define N6TLS_ERR_RNG           (-0x10003)  /* the random number generator failed  */
#define N6TLS_ERR_CRYPTO_INIT   (-0x10004)  /* psa_crypto_init() failed            */

/* Call once, from a task, before anything uses Mbed TLS. A repeat call
 * returns 0. */
int N6Tls_Init(void);

/* Time for certificate dates, in seconds since 1970-01-01 UTC (e.g.
 * MayaW476_GetTime()), then counted on from the tick; 0 forgets it.
 * N6Tls_GetTime() returns 0 while no time is known. */
void     N6Tls_SetTime(uint32_t UnixSeconds);
uint32_t N6Tls_GetTime(void);

/* The board's identity: a self-signed ECDSA P-256 key and certificate in DER
 * (as HTTPSRV_TLS_PARAM_STRUCT takes them), and the certificate's SHA-256
 * fingerprint for a client to compare. The key never leaves RAM and is new at
 * every start. */
#define N6TLS_CERT_MAX   1024U
#define N6TLS_KEY_MAX    256U

typedef struct
{
  uint8_t Cert[N6TLS_CERT_MAX];   /* the certificate, DER, CertLen bytes      */
  size_t  CertLen;
  uint8_t Key[N6TLS_KEY_MAX];     /* the private key, DER (SEC 1), KeyLen bytes */
  size_t  KeyLen;
  uint8_t Sha256[32];             /* SHA-256 of the certificate               */
} N6Tls_Identity;

/* Make a key and certificate for pDnsName (letters, digits, '-' and '.', up
 * to 63 characters; subject and alternative name), with Ipv4 as an
 * alternative name unless 0. Ipv4 is in lwIP's order (first octet in the low
 * byte), as MayaW476_LocalIP() gives it. Valid 2026-01-01 to 2049-12-31. */
int N6Tls_MakeIdentity(N6Tls_Identity *pId, const char *pDnsName, uint32_t Ipv4);

/* The certificate as PEM, for a client to import or pin; the fingerprint as
 * 32 colon-separated hex pairs (96 bytes with the terminator). */
int  N6Tls_CertPem(const N6Tls_Identity *pId, char *pBuf, size_t Size);
void N6Tls_Fingerprint(const N6Tls_Identity *pId, char *pBuf, size_t Size);

/* Zero the private key, once the server has taken its copy. */
void N6Tls_WipeKey(N6Tls_Identity *pId);

/* Client. N6Tls_Connect() resolves pHost, connects to Port and runs a TLS 1.3
 * or 1.2 handshake: the chain checked against pTrust (from
 * mbedtls_x509_crt_parse()), its dates against N6Tls_GetTime(), its names
 * against pHost; any failure fails the connection. TimeoutMs bounds each wait
 * for the server (connect, and every later read and write).
 * On success Ssl is the session's Mbed TLS context. On failure nothing needs
 * freeing, and VerifyFlags says why a chain was refused
 * (mbedtls_x509_crt_verify_info()). pTrust must outlive the connection. Call
 * N6Tls_Close() either way. */
typedef struct
{
  int                 Socket;       /* lwIP socket, -1 when there is none */
  uint32_t            TimeoutMs;
  uint32_t            VerifyFlags;  /* MBEDTLS_X509_BADCERT_* etc., 0 if not refused */
  uint8_t             Open;         /* handshake complete */
  mbedtls_ssl_config  Config;
  mbedtls_ssl_context Ssl;
} N6Tls_Client;

int N6Tls_Connect(N6Tls_Client *pClient, const char *pHost, uint16_t Port,
                  mbedtls_x509_crt *pTrust, uint32_t TimeoutMs);

/* Send all of pData, or fail. Returns Len or a negative code. */
int N6Tls_Write(N6Tls_Client *pClient, const void *pData, size_t Len);

/* Read up to Size bytes: the count, 0 after the server's close_notify, or a
 * negative code. An end without close_notify is MBEDTLS_ERR_SSL_CONN_EOF, so
 * a truncated reply cannot pass for whole; a protocol that gives the reply's
 * length (e.g. Content-Length) can accept it once complete. */
int N6Tls_Read(N6Tls_Client *pClient, void *pBuf, size_t Size);

/* Send close_notify if the session is open, close the socket and free
 * everything. Safe after a failed N6Tls_Connect(), and twice. */
void N6Tls_Close(N6Tls_Client *pClient);

/* Any code (this module, NXP's network layer, Mbed TLS) in words; returns
 * pBuf. */
const char *N6Tls_Strerror(int Err, char *pBuf, size_t Size);

#ifdef __cplusplus
}
#endif

#endif /* N6_TLS_H */
