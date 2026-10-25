# Neuro N6 TLS

TLS over Wi-Fi, built from Mbed TLS (`Middlewares/mbedtls`), NXP's HTTP server
and its Mbed TLS layer (`Middlewares/lwip/src/apps/httpsrv`), and NXP's Mbed
TLS transport over lwIP's sockets (`wm_mbedtls_net.c`, here). This folder adds
what those need from the board: entropy, mutexes, the time, a key and
certificate for the board, and a client connection that checks its server.

| file | what it is |
|---|---|
| `n6_tls.{h,c}` | `N6Tls_Init()` (TRNG entropy, FreeRTOS mutexes, PSA), `N6Tls_SetTime()`, `N6Tls_MakeIdentity()` (key and self-signed certificate), and a verified client: `N6Tls_Connect()`, `_Write()`, `_Read()`, `_Close()` |
| `wm_mbedtls_net.c` | NXP's transport, verbatim from NXP's `wifi_nxp` (`wmcrypto/`); its header is `Middlewares/NXP/wifi_nxp/wmcrypto/wm_mbedtls_net.h` |
| `include/mbedtls_config_tls.h` | Mbed TLS for TLS 1.2 and 1.3, server and client |
| `include/threading_alt.h` | Mbed TLS's mutex type under FreeRTOS |

## Using it

A server, as `Examples/WiFiTls_FreeRTOS` runs one:

```c
#include "n6_tls.h"
#include "httpsrv.h"

N6Tls_Init();                                        /* once, from a task */
N6Tls_MakeIdentity(&Id, "neuron6-a1b2c3.local", MayaW476_LocalIP());

tls.certificate_buffer      = Id.Cert;               /* HTTPSRV_TLS_PARAM_STRUCT */
tls.certificate_buffer_size = Id.CertLen;
tls.private_key_buffer      = Id.Key;
tls.private_key_buffer_size = Id.KeyLen;
params.tls_param            = &tls;                  /* HTTPSRV_PARAM_STRUCT */
Server = HTTPSRV_init(&params);                      /* NXP's server, on 443 */
N6Tls_WipeKey(&Id);                                  /* it keeps its own copy */
```

A client:

```c
mbedtls_x509_crt_parse(&Trust, (const unsigned char *)RootsPem, sizeof(RootsPem));
N6Tls_SetTime(MayaW476_GetTime());                   /* NTP; nothing is checked without it */

if (N6Tls_Connect(&c, "example.org", 443, &Trust, 10000) == 0)
{
  N6Tls_Write(&c, request, len);
  while ((n = N6Tls_Read(&c, buf, sizeof(buf))) > 0) { ... }
}
N6Tls_Close(&c);
```

The rest is Mbed TLS's API (`mbedtls_ssl_get_version()`,
`mbedtls_ssl_get_peer_cert()` on `c.Ssl`) and NXP's (`httpsrv.h`: CGI, SSI,
authentication, the ROM file system).

The build needs the sources, include paths and switches listed in
`Examples/WiFiTls_FreeRTOS/tls_build.json`, the Wi-Fi stack of its
`wifi_build.json`, the HAL's random number generator driver
(`HAL_RNG_MODULE_ENABLED`), and FreeRTOS.

## What is NXP's

Taken: the HTTP server (`httpsrv`, BSD-3-Clause, with its ROM file system
generator `mkfs/mkfs.pl` and `httpsrv_tls.c`), and the transport, unmodified.
The server has three changes, each marked `Ohm Lab:` in the source:

- `HTTPSRV_CFG_LISTEN_BACKLOG`, default NXP's 0, which lwIP makes one; the
  example asks for 2. With one place, a connection abandoned before it
  completes holds the listener for lwIP's 20 s SYN-RCVD timeout. Each waiting
  connection takes one of lwIP's eight TCP connections (`MEMP_NUM_TCP_PCB`).
- `HTTPSRV_CFG_TLS_HANDSHAKE_TIMEOUT`, in milliseconds, 0 (none) by default;
  the example sets 10 s. It limits a whole handshake, which the listener runs
  itself, so a client sending a byte at a time cannot hold it.
- A CGI's addresses start empty, as `getsockname()` and `getpeername()` fill
  in nothing after a peer reset.

Not taken: `wm_mbedtls_entropy.c` (seeds from `rand()`, so its keys would be
predictable); `wm_mbedtls_helper_api.c` and `wm_mbedtls_mem.c` (need that
entropy, accept certificates signed with SHA-224 or RIPEMD-160, predate
Mbed TLS 3); lwIP's `altcp_tls` and HTTP client (they run the handshake
inside the TCP/IP thread).

## How it runs

- **Entropy.** The TRNG, with clock error detection, seeds the PSA API's
  CTR_DRBG through `mbedtls_hardware_poll()`.
- **Time.** `N6Tls_SetTime()`, usually from `MayaW476_GetTime()`, then
  counted on from the tick. Mbed TLS never asks the network. With no time
  set, `N6Tls_Connect()` refuses before connecting (`N6TLS_ERR_NO_CLOCK`).
- **The identity.** An ECDSA P-256 key and a self-signed X.509 v3
  certificate: subject `O=Neuro N6, CN=<name>`, the DNS name and address as
  alternative names, not a CA, TLS server only, a random 16-byte serial,
  valid 2026-01-01 to 2049-12-31. A client pins it by its SHA-256
  fingerprint, or trusts it explicitly.
- **The client.** The chain is checked with Mbed TLS's default profile
  (SHA-256 or better, RSA of 2048 bits or more), its dates against the
  glue's clock, its name or address against the host, which is also sent as
  the server name.
- **The server.** The listener runs each handshake, then hands the session to
  its own task at FreeRTOS priority 1, below the TCP/IP thread.
- **Memory.** Mbed TLS uses the C library's heap, NXP's server the FreeRTOS
  heap. An open session takes 25,896 bytes of the C heap and 12,400 of the
  FreeRTOS heap, most of it the task's 10 KB stack.
- **Uploads** need lwIP's receive mailbox (`DEFAULT_TCP_RECVMBOX_SIZE` in
  `Middlewares/lwip/port/lwipopts.h`) to hold a whole receive window. With
  room for 12 of the window's 44 segments, lwIP dropped about a quarter of an
  upload's segments and uploads ran at 93 to 124 KB/s.

`include/mbedtls_config_tls.h` (`MBEDTLS_CONFIG_FILE="mbedtls_config_tls.h"`)
allows TLS 1.2 with ECDHE only, TLS 1.3 without pre-shared keys, tickets or
early data, AES-GCM and ChaCha20-Poly1305 only, and SHA-256 and SHA-384:
certificates signed with SHA-1 or MD5 are refused. Its comments give the
reasons.

## NXP's code, as it behaves here

- `wm_mbedtls_net.c` reports a timed-out read or write as
  `MBEDTLS_ERR_SSL_WANT_READ` or `_WANT_WRITE` (it takes a non-zero
  `lwip_fcntl(F_GETFL)`, which lwIP 2.2 always returns, as non-blocking), so
  the glue ends an operation at its deadline.
- **Keep-alive lasts a second.** lwIP leaves `SO_ERROR` at 0 after a receive
  timeout, so an idle connection ends at the server's 1 s receive timeout,
  whatever `HTTPSRV_CFG_KEEPALIVE_TIMEOUT` says.
- A client slow to finish its handshake holds up the next connection: by up
  to 1 s on each read, and by up to the handshake time limit in all.
- A CGI name the table does not have gets no reply; the connection just ends.
- `wm_mbedtls_net.c` gives three format warnings in the build.

Tested on a Neuro N6 (CubeIDE's Debug build, `-O0`) against Python's `ssl`
and OpenSSL on a PC: 94 of 94 checks; server handshakes averaged 282 ms, and
1 MB moved at 585 KB/s from the board and 612 KB/s to it.

## Limits

- **No revocation checking** (neither OCSP nor CRLs).
- **A new identity at every start**, by design: a client that pinned the
  certificate has to pin the new one after a reset.
- **Resumption in TLS 1.2 only**: TLS 1.3 session tickets are configured out.
- **A TLS 1.2 server that signs with RSA-PSS is refused**: Mbed TLS 3.6.6
  accepts PSS only in TLS 1.3. TLS 1.3 servers with RSA keys, and TLS 1.2
  servers signing with PKCS#1 v1.5, work.
- **An address as host name** goes into the server name extension too, where
  RFC 6066 allows only names; OpenSSL-based servers accept it.
- **A client that stops reading holds the network's memory**: unsent data
  waits in lwIP's 96 KB heap, up to 64 KB for each connection
  (`TCP_SND_BUF`), and NXP's server sets no send timeout
  (`HTTPSRV_CFG_SEND_TIMEOUT`). Two such clients can exhaust it.
- The client's timeout bounds each wait, not the whole exchange, and not the
  name lookup, which lwIP's DNS client bounds with its own retries.
- FreeRTOS only.

## Licence

`n6_tls.{h,c}`, `include/mbedtls_config_tls.h` and `include/threading_alt.h`
are Ohm Lab Ltd's, BSD 3-Clause, as the rest of the BSP. `wm_mbedtls_net.c`
is NXP's, BSD 3-Clause, as its header states. Mbed TLS and NXP's HTTP server
keep their own licences; see `THIRD-PARTY-NOTICES.md`.
