# lwIP

The lwIP TCP/IP stack, **2.2.1** (`src/include/lwip/init.h`), with NXP's port
for FreeRTOS, copied verbatim. The PPP implementation is not carried; see
`THIRD-PARTY-NOTICES.md` section 2.9.

| folder | what it is |
|---|---|
| `src/` | lwIP itself: the core, IPv4 and IPv6, the sequential and socket APIs, the network interfaces |
| `src/apps/httpsrv/` | NXP's HTTP and HTTPS server, from the same NXP distribution of lwIP: listener and session tasks over lwIP's sockets, CGI, server-side includes, basic authentication, a ROM file system (`mkfs/mkfs.pl` generates one from a folder of web files), and TLS through Mbed TLS (`httpsrv_tls.c`). Three changes, marked `Ohm Lab:` in the source: the listen backlog is an option, `HTTPSRV_CFG_LISTEN_BACKLOG`, NXP's value by default; so is a time limit on a whole TLS handshake, `HTTPSRV_CFG_TLS_HANDSHAKE_TIMEOUT`, none by default; and a CGI's addresses start empty. lwIP's own apps are not carried |
| `port/` | NXP's port: `lwipopts.h`, the configuration the NXP Wi-Fi driver is tuned for (sockets, DHCP, DNS, IGMP, an RTOS); `arch/`, the compiler and platform definitions; `sys_arch/dynamic/`, the FreeRTOS glue. This repository's changes - the TCP/IP thread's stack, the stream's throughput settings, lwIP's own DNS protections and the random numbers they need - are marked `Ohm Lab:` in the source |

lwIP is under its BSD-3-Clause licence (`COPYING`), and NXP's port and HTTP
server files under BSD-3-Clause, as their headers state. The Wi-Fi driver's
configuration, `Middlewares/NeuroN6/MayaW476/maya_w4_board/wifi_config.h`,
selects lwIP's high-throughput profile and turns IPv6 off.

The Wi-Fi examples compile `src/api`, `src/core`, `src/netif` and `port`.
Only `Examples/WiFiTls_FreeRTOS` compiles the HTTP server,
configured by its `tls_build.json`; `Middlewares/NeuroN6/Tls/README.md`
describes how the server behaves on this lwIP.
