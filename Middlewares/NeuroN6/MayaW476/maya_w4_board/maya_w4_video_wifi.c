/**
 ******************************************************************************
 * @file    maya_w4_video_wifi.c
 * @brief   The Wi-Fi end of the Neuro host link: a TCP server that hands each
 *          client to the application, and a UDP discovery responder, on the
 *          board's own access point only.
 *
 * An accepted client belongs to the USB task (NeuroN6_NetAttach), which drives
 * it like the USB vendor link (framing and discovery in n6_host_contract.h).
 * One client at a time: the listener is closed while one is attached. Send and
 * recv never block: blocking in lwip_send wedges wlcmgr_task. With a link key
 * set, a client must first answer a challenge; up to three wait, each for
 * N6_LINK_ANSWER_MS at most.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/MayaW476/LICENSE-MIT.txt.
 */

#include "FreeRTOS.h"
#include "task.h"

#include "NeuroN6_FrameQueue.h"   /* n6_net_link_t, NeuroN6_NetAttach, NeuroN6_IpAnnounceFrame */
#include "n6_host_contract.h"     /* N6_FRAME_MAGIC, n6_frame_hdr_t */
#include "maya_w4_creds.h"        /* tcp port, link state */
#include "maya_w4_sha256.h"       /* the link key's HMAC */
#include "stm32n6xx_hal.h"        /* DWT->CYCCNT, HAL_GetUIDw0() */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* lwIP BSD socket API */
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* CDC debug print (see maya_w4_hw.c). */
extern void NeuroN6_DbgPrint(const char *msg);

/* ms->ticks delay helper. */
#define MAYA_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))

/* Two, though one host is served at a time: a half-open connection its host
 * abandoned must not hold the only place for lwIP's 20 s timeout. */
#define WIFI_TCP_BACKLOG    2
#define WIFI_POLL_MS        100    /* socket wait, and how soon a released client closes */

/* TCP keepalive, so a vanished host is noticed while idle: probes after 5 s
 * of silence, one a second, given up after 5. */
#define WIFI_KEEPIDLE_S     5
#define WIFI_KEEPINTVL_S    1
#define WIFI_KEEPCNT        5

/* State. */
static TaskHandle_t  s_task          = NULL;
static uint16_t      s_tcp_port      = 5000;
static int           s_client        = -1;      /* the attached client, or -1 */
static volatile bool s_client_closed = false;   /* the USB task has let go of it */

/* Clients proving the link key, one place per address, so a peer that never
 * answers cannot keep out a host that knows the key. A newcomer takes its
 * address's place, else a free one, else the oldest. The first to prove the
 * key is attached; the rest are refused. */
#define LINK_PENDING_MAX    3u

typedef struct {
  int        fd;                                       /* -1: a free place */
  uint32_t   addr;                                     /* the peer's address */
  TickType_t since;
  uint8_t    challenge[N6_LINK_CHALLENGE_BYTES];
  uint8_t    answer[sizeof(n6_frame_hdr_t) + 4u + 32u];
  uint32_t   got;
} link_pending_t;

static link_pending_t s_pending[LINK_PENDING_MAX];
static uint32_t       s_challenges    = 0u;

/* The client, as the USB task drives it. */

/* 0 means full. A failed connection is reported by client_recv instead:
 * lwIP's errno is one global shared by every task. */
static int32_t client_send(const uint8_t *data, uint32_t len)
{
  const int n = lwip_send(s_client, data, len, MSG_DONTWAIT);
  return (n > 0) ? (int32_t)n : 0;
}

/* A zero-timeout select says whether anything is waiting: data, the peer's
 * close, or an error. Readable but nothing read means the connection is gone. */
static int32_t client_recv(uint8_t *data, uint32_t len)
{
  if (len == 0u) return 0;

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(s_client, &rd);
  struct timeval now = { 0, 0 };
  if (lwip_select(s_client + 1, &rd, NULL, NULL, &now) <= 0) return 0;

  const int n = lwip_recv(s_client, data, len, MSG_DONTWAIT);
  return (n > 0) ? (int32_t)n : -1;
}

/* The USB task will not touch the client again; this task closes it. */
static void client_closed(void)
{
  s_client_closed = true;
}

static const n6_net_link_t s_link = { client_send, client_recv, client_closed };

/* Sockets. */

/* Zero-linger close: a host dropped for not reading can hold TCP_SND_BUF
 * (64 KB) of lwIP's 96 KB heap through a graceful close. With data queued lwIP
 * resets and frees at once; with none it still closes normally. */
static void close_client(int fd)
{
  const struct linger now = { 1, 0 };
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_LINGER, &now, sizeof(now));
  lwip_close(fd);
}

/* Refuse with a reset. lwIP resets on a zero-linger close only with data
 * queued (otherwise a peer can hold FIN_WAIT for minutes), so queue a byte. */
static void reset_client(int fd)
{
  const uint8_t one = 0u;
  (void)lwip_send(fd, &one, 1u, MSG_DONTWAIT);
  close_client(fd);
}

static int open_listen_socket(void)
{
  int s = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) return -1;

  int yes = 1;
  (void)lwip_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  (void)memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);   /* bind all interfaces */
  addr.sin_port        = htons(s_tcp_port);

  if (lwip_bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    lwip_close(s);
    return -1;
  }
  if (lwip_listen(s, WIFI_TCP_BACKLOG) < 0) {
    lwip_close(s);
    return -1;
  }
  return s;
}

/* Discovery probes arrive on the stream's port number, over UDP. */
static int open_probe_socket(void)
{
  int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return -1;

  struct sockaddr_in addr;
  (void)memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(s_tcp_port);

  if (lwip_bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    lwip_close(s);
    return -1;
  }
  return s;
}

/* Low latency, liveness checks, and never block. */
static void client_setup(int fd)
{
  int on = 1;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  int v = WIFI_KEEPIDLE_S;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &v, sizeof(v));
  v = WIFI_KEEPINTVL_S;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &v, sizeof(v));
  v = WIFI_KEEPCNT;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &v, sizeof(v));
  (void)lwip_ioctl(fd, FIONBIO, &on);
}

/* mDNS responder (maya_w4_mdns.c), served from this task's select loop. */
int  maya_w4_mdns_open(void);
void maya_w4_mdns_announce(int s);
void maya_w4_mdns_answer(int s);
void maya_w4_mdns_close(int s);

/* The discovery reply: a META_BOARD_IPV4 frame, then the board's name
 * (META_BOARD_NAME), so a host that hears several boards can tell them apart. */
static uint32_t probe_reply(uint8_t *out, uint32_t cap)
{
  uint32_t len = NeuroN6_IpAnnounceFrame(out, cap);
  if (len == 0u) return 0u;

  const char *name = maya_w4_get_hostname();
  size_t nl = strlen(name);
  if (nl > 32u) nl = 32u;
  if (len + 3u + nl <= cap) {
    out[len]      = N6_META_BOARD_NAME;
    out[len + 1u] = (uint8_t)nl;
    out[len + 2u] = 0u;
    memcpy(&out[len + 3u], name, nl);
    len += 3u + (uint32_t)nl;
  }
  uint8_t key[64];
  if (maya_w4_get_link_key(key, sizeof(key)) > 0u && len + 4u <= cap) {
    out[len]      = N6_META_BOARD_LINK;
    out[len + 1u] = 1u;
    out[len + 2u] = 0u;
    out[len + 3u] = N6_BOARD_LINK_KEYED;
    len += 4u;
  }
  maya_w4_wipe(key, sizeof(key));

  /* meta_len covers every TLV after the header. */
  const uint32_t meta = len - (uint32_t)sizeof(n6_frame_hdr_t);
  memcpy(&out[8], &meta, sizeof(meta));
  return len;
}

/* Answer a discovery probe, to the prober only. Ignores anything that is not
 * a frame, and probes from off the board's subnet. */
static void answer_probe(int udp)
{
  uint8_t in[64];
  struct sockaddr_in from;
  socklen_t from_len = (socklen_t)sizeof(from);
  const int n = lwip_recvfrom(udp, in, sizeof(in), MSG_DONTWAIT,
                              (struct sockaddr *)&from, &from_len);
  if (n < (int)sizeof(n6_frame_hdr_t)) return;
  if (!maya_w4_wifi_on_link(from.sin_addr.s_addr)) return;

  uint32_t magic;
  memcpy(&magic, in, sizeof(magic));
  if (magic != N6_FRAME_MAGIC) return;

  uint8_t out[96];
  const uint32_t len = probe_reply(out, sizeof(out));
  if (len != 0u) {
    (void)lwip_sendto(udp, out, len, 0, (struct sockaddr *)&from, from_len);
  }
}

/* The link key. */

/* A challenge that never repeats (chip ID, count, cycle counter, tick and
 * peer, hashed), so an overheard answer is no use again. */
static void make_challenge(link_pending_t *p, const struct sockaddr_in *peer)
{
  maya_w4_sha256_t c;
  uint8_t d[32];
  const uint32_t mix[6] = {
    HAL_GetUIDw0(), HAL_GetUIDw1() ^ HAL_GetUIDw2(), ++s_challenges,
    DWT->CYCCNT, (uint32_t)xTaskGetTickCount(),
    peer->sin_addr.s_addr ^ ((uint32_t)peer->sin_port << 16),
  };
  maya_w4_sha256_init(&c);
  maya_w4_sha256_update(&c, mix, sizeof(mix));
  maya_w4_sha256_final(&c, d);
  memcpy(p->challenge, d, sizeof(p->challenge));
}

/* The challenge frame: a header and one TLV, nothing else. */
static bool send_challenge(const link_pending_t *p)
{
  uint8_t f[sizeof(n6_frame_hdr_t) + 3u + N6_LINK_CHALLENGE_BYTES];
  n6_frame_hdr_t h = { N6_FRAME_MAGIC, 0u, 3u + N6_LINK_CHALLENGE_BYTES, 0u, 0u };
  memcpy(f, &h, sizeof(h));
  f[sizeof(h)]      = N6_META_BOARD_CHALLENGE;
  f[sizeof(h) + 1u] = N6_LINK_CHALLENGE_BYTES;
  f[sizeof(h) + 2u] = 0u;
  memcpy(&f[sizeof(h) + 3u], p->challenge, N6_LINK_CHALLENGE_BYTES);
  return lwip_send(p->fd, f, sizeof(f), MSG_DONTWAIT) == (int)sizeof(f);
}

/* 1: the right answer. 0: not all of it yet. -1: wrong, or not an answer. */
static int check_answer(const link_pending_t *p)
{
  n6_frame_hdr_t h;
  if (p->got < sizeof(h)) return 0;
  memcpy(&h, p->answer, sizeof(h));
  if (h.magic != N6_FRAME_MAGIC || h.meta_len != 0u || h.payload_len != 4u + 32u) return -1;
  if (p->got < sizeof(p->answer)) return 0;

  uint32_t magic;
  memcpy(&magic, &p->answer[sizeof(h)], sizeof(magic));
  if (magic != N6_CMD_KEY0) return -1;

  uint8_t key[64], want[32];
  const size_t kn = maya_w4_get_link_key(key, sizeof(key));
  maya_w4_hmac_sha256(key, kn, N6_LINK_CONTEXT, sizeof(N6_LINK_CONTEXT) - 1u,
                      p->challenge, sizeof(p->challenge), want);
  uint8_t diff = (kn == 0u) ? 1u : 0u;
  for (uint32_t i = 0u; i < 32u; i++) diff |= (uint8_t)(want[i] ^ p->answer[sizeof(h) + 4u + i]);
  maya_w4_wipe(key, sizeof(key));
  maya_w4_wipe(want, sizeof(want));
  return (diff == 0u) ? 1 : -1;
}

/* Refuse a connection still proving the key, with a reset: a peer ignoring a
 * close would hold one of lwIP's eight TCP connections for minutes. */
static void drop_pending(link_pending_t *p, const char *why)
{
  if (p->fd < 0) return;
  reset_client(p->fd);
  p->fd = -1;
  if (why != NULL) {
    char dbg[112];
    (void)snprintf(dbg, sizeof(dbg), "[MayaW476] a Neuro host was refused: %s.\r\n", why);
    NeuroN6_DbgPrint(dbg);
  }
}

/* A client gets the stream: hand it to the USB task. */
static void attach(int fd)
{
  s_client_closed = false;
  s_client = fd;
  if (!NeuroN6_NetAttach(&s_link)) {
    s_client = -1;
    lwip_close(fd);
    NeuroN6_DbgPrint("[MayaW476] the USB task could not take the host - closed.\r\n");
    return;
  }
  NeuroN6_DbgPrint("[MayaW476] Neuro host connected - stream and controls on WiFi.\r\n");
}

static void drop_all_pending(const char *why)
{
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) drop_pending(&s_pending[i], why);
}

/* Where a newcomer from `addr` waits: its address's place, a free one, or
 * the place of the one that has waited longest. */
static link_pending_t *pending_place(uint32_t addr)
{
  link_pending_t *free_place = NULL, *oldest = &s_pending[0];
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
    link_pending_t *p = &s_pending[i];
    if (p->fd >= 0 && p->addr == addr) return p;
    if (p->fd < 0) {
      if (free_place == NULL) free_place = p;
    } else if (oldest->fd >= 0 && (TickType_t)(p->since - oldest->since) > (TickType_t)(1UL << 31)) {
      oldest = p;
    }
  }
  return (free_place != NULL) ? free_place : oldest;
}

/* A client proving the key sent something, or went away. True once it has
 * proved it and been attached. */
static bool pending_readable(link_pending_t *p)
{
  const int n = lwip_recv(p->fd, &p->answer[p->got], sizeof(p->answer) - p->got, MSG_DONTWAIT);
  if (n <= 0) { drop_pending(p, "it left without the link key"); return false; }
  p->got += (uint32_t)n;
  const int r = check_answer(p);
  if (r < 0) { drop_pending(p, "wrong link key"); return false; }
  if (r == 0) return false;

  const int fd = p->fd;
  p->fd = -1;
  drop_all_pending("another host proved the link key first");
  NeuroN6_DbgPrint("[MayaW476] link key proved.\r\n");
  attach(fd);
  return true;
}

/* Task. */

/* Link down: shut the client down (its next recv reports the host gone) and
 * wait for the USB task to let go, so its socket number is not reused under it. */
static void release_client(void)
{
  if (s_client < 0) return;
  (void)lwip_shutdown(s_client, SHUT_RDWR);
  for (uint32_t i = 0U; !s_client_closed; ++i) {
    MAYA_DELAY_MS(50U);
    if ((i % 40U) == 39U) (void)lwip_shutdown(s_client, SHUT_RDWR);
  }
  close_client(s_client);
  s_client = -1;
  NeuroN6_DbgPrint("[MayaW476] Neuro host disconnected - link down.\r\n");
}

/* The server runs while the board hosts its own access point. */
static bool ap_link_ready(void)
{
  return maya_w4_wifi_link_ready() && (maya_w4_wifi_mode() == MAYA_W4_MODE_AP);
}

/* Serve one link: from the access point coming up until it goes down or
 * changes. */
static void serve_link(uint32_t generation)
{
  int probe_fd  = open_probe_socket();
  int listen_fd = -1;
  int mdns_fd   = maya_w4_mdns_open();
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) s_pending[i].fd = -1;
  maya_w4_mdns_announce(mdns_fd);
  const TickType_t up_at = xTaskGetTickCount();
  bool announced_twice = false;
  {
    char dbg[160];
    (void)snprintf(dbg, sizeof(dbg),
                   "[MayaW476] WiFi up - Neuro host server on TCP :%u, discovery on "
                   "UDP :%u%s, mDNS %s.local\r\n", (unsigned)s_tcp_port, (unsigned)s_tcp_port,
                   (probe_fd < 0) ? " (FAILED)" : "", maya_w4_get_hostname());
    NeuroN6_DbgPrint(dbg);
  }

  while (ap_link_ready() && maya_w4_wifi_link_generation() == generation) {
    /* The USB task let go of the client: close it, then listen again on a fresh
     * socket, so a wedged lwIP PCB cannot jam the port. */
    if (s_client >= 0 && s_client_closed) {
      close_client(s_client);
      s_client = -1;
      NeuroN6_DbgPrint("[MayaW476] Neuro host disconnected - listening again.\r\n");
    }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      if (s_pending[i].fd >= 0 &&
          (xTaskGetTickCount() - s_pending[i].since) > pdMS_TO_TICKS(N6_LINK_ANSWER_MS)) {
        drop_pending(&s_pending[i], "no answer to the link key challenge");
      }
    }
    if (s_client < 0 && listen_fd < 0) {
      listen_fd = open_listen_socket();
      if (listen_fd < 0) { MAYA_DELAY_MS(1000U); continue; }
    }

    /* RFC 6762 asks for a second announcement a second after the first. */
    if (!announced_twice && (xTaskGetTickCount() - up_at) > pdMS_TO_TICKS(1000U)) {
      maya_w4_mdns_announce(mdns_fd);
      announced_twice = true;
    }

    fd_set rd;
    FD_ZERO(&rd);
    int maxfd = -1;
    if (probe_fd >= 0)  { FD_SET(probe_fd, &rd);  maxfd = probe_fd; }
    if (listen_fd >= 0) { FD_SET(listen_fd, &rd); if (listen_fd > maxfd) maxfd = listen_fd; }
    if (mdns_fd >= 0)   { FD_SET(mdns_fd, &rd);   if (mdns_fd > maxfd)   maxfd = mdns_fd; }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      const int fd = s_pending[i].fd;
      if (fd >= 0) { FD_SET(fd, &rd); if (fd > maxfd) maxfd = fd; }
    }
    if (maxfd < 0) { MAYA_DELAY_MS(WIFI_POLL_MS); continue; }

    struct timeval wait = { 0, WIFI_POLL_MS * 1000 };
    if (lwip_select(maxfd + 1, &rd, NULL, NULL, &wait) <= 0) continue;

    if (probe_fd >= 0 && FD_ISSET(probe_fd, &rd)) {
      answer_probe(probe_fd);
    }
    if (mdns_fd >= 0 && FD_ISSET(mdns_fd, &rd)) {
      maya_w4_mdns_answer(mdns_fd);
    }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      if (s_pending[i].fd >= 0 && FD_ISSET(s_pending[i].fd, &rd) &&
          pending_readable(&s_pending[i]) && listen_fd >= 0) {
        lwip_close(listen_fd);   /* one host at a time: refuse others meanwhile */
        listen_fd = -1;
      }
    }

    if (listen_fd >= 0 && FD_ISSET(listen_fd, &rd)) {
      struct sockaddr_in peer;
      socklen_t peer_len = (socklen_t)sizeof(peer);
      const int fd = lwip_accept(listen_fd, (struct sockaddr *)&peer, &peer_len);
      if (fd < 0) continue;

      client_setup(fd);
      uint8_t key[64];
      const size_t keyed = maya_w4_get_link_key(key, sizeof(key));
      maya_w4_wipe(key, sizeof(key));
      if (keyed == 0u) {
        lwip_close(listen_fd);   /* one host at a time: refuse others meanwhile */
        listen_fd = -1;
        attach(fd);
        continue;
      }
      /* The listener stays open until a host has proved the key. */
      link_pending_t *p = pending_place(peer.sin_addr.s_addr);
      drop_pending(p, "a newer connection took its place");
      p->fd    = fd;
      p->addr  = peer.sin_addr.s_addr;
      p->since = xTaskGetTickCount();
      p->got   = 0u;
      make_challenge(p, &peer);
      if (!send_challenge(p)) drop_pending(p, "the challenge could not be sent");
    }
  }

  drop_all_pending(NULL);
  release_client();
  if (listen_fd >= 0) lwip_close(listen_fd);
  if (probe_fd >= 0)  lwip_close(probe_fd);
  maya_w4_mdns_close(mdns_fd);
}

static void maya_w4_video_wifi_task(void *arg)
{
  (void)arg;

  for (;;) {
    /* Wait for the board's own access point, with DHCP. Until then, and
     * between links, the stream stays on USB. */
    bool told = false;
    for (uint32_t i = 0U; !ap_link_ready(); ++i) {
      if (i == 600U && !told) {
        NeuroN6_DbgPrint("[MayaW476] The board's access point is not up - the stream stays "
                         "on USB.\r\n");
        told = true;
      }
      MAYA_DELAY_MS(100U);
    }
    const uint32_t generation = maya_w4_wifi_link_generation();
    MAYA_DELAY_MS(200U);   /* settle */
    serve_link(generation);
    NeuroN6_DbgPrint("[MayaW476] WiFi link down - host server waiting for the next one.\r\n");
  }
}

/* Public entry - spawn the server task. Idempotent. */
int maya_w4_video_wifi_start(uint16_t tcp_port)
{
  if (s_task != NULL) {
    return 0;  /* already running */
  }

  s_tcp_port = (tcp_port != 0U) ? tcp_port : maya_w4_get_tcp_port();

  /* 6 kB (1536 words): the BSD socket calls use ~1.5-2 kB. */
  BaseType_t ok = xTaskCreate(maya_w4_video_wifi_task,
                              "maya_w4_vid",
                              1536U,
                              NULL,
                              tskIDLE_PRIORITY + 3,   /* above the connect task */
                              &s_task);
  if (ok != pdPASS) {
    s_task = NULL;
    NeuroN6_DbgPrint("[MayaW476] host server task create FAILED.\r\n");
    return -1;
  }
  return 0;
}
