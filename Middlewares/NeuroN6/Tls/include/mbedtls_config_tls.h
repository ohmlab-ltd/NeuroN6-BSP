/**
  ******************************************************************************
  * @file    mbedtls_config_tls.h
  * @brief   Mbed TLS for TLS over Wi-Fi: TLS 1.2 and 1.3, server and client,
  *          authenticated encryption only, certificates checked against a
  *          trust store, key and certificate made on the board.
  *          Select with MBEDTLS_CONFIG_FILE="mbedtls_config_tls.h".
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

#ifndef MBEDTLS_CONFIG_TLS_H
#define MBEDTLS_CONFIG_TLS_H

#include <time.h>

/* Platform */

/* Thumb-2 bignum assembly only when optimising: unoptimised GCC (CubeIDE's
 * Debug build) cannot allocate its registers. */
#if defined(__OPTIMIZE__)
#define MBEDTLS_HAVE_ASM
#endif

/* C library calloc/free (heap locked by neuron6_rtos.c). An open session
 * takes 25,896 bytes. */
#define MBEDTLS_PLATFORM_C

/* Certificate time: N6Tls_SetTime()'s value counted on from the tick, 0 until
 * set. Until then a client refuses every server; N6Tls_Connect() says so. */
#define MBEDTLS_HAVE_TIME
#define MBEDTLS_HAVE_TIME_DATE
#define MBEDTLS_PLATFORM_TIME_MACRO     N6Tls_MbedtlsTime
#define MBEDTLS_PLATFORM_MS_TIME_ALT
time_t N6Tls_MbedtlsTime(time_t *pTime);

/* The STM32N6 TRNG is the only entropy source (mbedtls_hardware_poll() in
 * n6_tls.c); it seeds the generator behind every key and nonce. */
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_CTR_DRBG_C

/* Server sessions, a client and the connection manager may run at once:
 * FreeRTOS mutexes (threading_alt.h, n6_tls.c). */
#define MBEDTLS_THREADING_C
#define MBEDTLS_THREADING_ALT

/* PSA cryptography: needed by TLS 1.3. psa_crypto_init() runs in N6Tls_Init();
 * keys in RAM only. No MBEDTLS_USE_PSA_CRYPTO: with STM32CubeN6 v1.4.1's
 * Mbed TLS it does not link (mbedtls_md_error_from_psa() needs a hardware
 * hash driver, MBEDTLS_MD_SOME_PSA). */
#define MBEDTLS_PSA_CRYPTO_C

/* Algorithms */

/* AES-GCM and ChaCha20-Poly1305 only: no CBC, nothing unauthenticated. */
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES
#define MBEDTLS_GCM_C
#define MBEDTLS_CHACHA20_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_CHACHAPOLY_C
#define MBEDTLS_CIPHER_C

/* SHA-256, SHA-384 and HKDF. No SHA-1 or MD5: certificates signed with
 * either are refused. */
#define MBEDTLS_MD_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA384_C
#define MBEDTLS_SHA512_C
#define MBEDTLS_HKDF_C
#define MBEDTLS_HMAC_DRBG_C

/* P-256 (the board's key), P-384 (some chains), X25519 (most clients' first
 * offer). Deterministic ECDSA (RFC 6979). */
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECP_NIST_OPTIM
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECDSA_DETERMINISTIC

/* RSA for checking peers only (the board makes no RSA key): PKCS#1 v1.5 for
 * certificates, PSS for TLS 1.3 handshakes. Without
 * MBEDTLS_X509_RSASSA_PSS_SUPPORT no RSA-keyed server completes TLS 1.3. */
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_X509_RSASSA_PSS_SUPPORT

/* Keys and certificates: parse the trust store and peer chains (DER, PEM);
 * write the board's key and self-signed certificate, and print it as PEM. */
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_OID_C
#define MBEDTLS_BASE64_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_PEM_WRITE_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_PK_WRITE_C
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_CREATE_C
#define MBEDTLS_X509_CRT_WRITE_C

/* TLS */

#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_SRV_C
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_PROTO_TLS1_3

/* TLS 1.2: ECDHE only (forward secrecy), signed with ECDSA (the board's
 * certificate) or RSA (servers it connects to). */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
#define MBEDTLS_SSL_EXTENDED_MASTER_SECRET

/* TLS 1.3: ephemeral full handshakes; no pre-shared keys, session tickets or
 * early data. Middlebox compatibility mode. */
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED
#define MBEDTLS_SSL_TLS1_3_COMPATIBILITY_MODE

/* SNI; the peer's certificate kept for the application (TLS 1.3 requires
 * it); every alert sent, so a failing peer learns why. */
#define MBEDTLS_SSL_SERVER_NAME_INDICATION
#define MBEDTLS_SSL_KEEP_PEER_CERTIFICATE
#define MBEDTLS_SSL_ALL_ALERT_MESSAGES

/* TLS 1.2 session resumption from the server's cache (NXP's HTTPS server). */
#define MBEDTLS_SSL_CACHE_C
#define MBEDTLS_SSL_CACHE_DEFAULT_MAX_ENTRIES   8

/* Incoming records (up to 16 KB) are taken whole. Outgoing stay well under
 * 4 KB: NXP's server writes 1360 bytes at a time. */
#define MBEDTLS_SSL_IN_CONTENT_LEN              16384
#define MBEDTLS_SSL_OUT_CONTENT_LEN             4096

/* Diagnostics: mbedtls_strerror() names every error; the version is
 * printable. */
#define MBEDTLS_ERROR_C
#define MBEDTLS_VERSION_C

#endif /* MBEDTLS_CONFIG_TLS_H */
