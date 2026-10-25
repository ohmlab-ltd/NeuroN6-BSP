/**
 ******************************************************************************
 * @file    maya_w4_mdns.c
 * @brief   mDNS responder (RFC 6762) with DNS-SD (RFC 6763): answers to
 *          <hostname>.local and advertises _neuron6._tcp, the Neuro Studio
 *          host server, with its port.
 *
 * Driven by the host server task's select loop (maya_w4_video_wifi.c).
 * Answers A, PTR, SRV, TXT and ANY, legacy unicast queries included, and
 * announces twice on link-up. No conflict probing: the default name carries
 * 24 bits of the chip's unique ID.
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "maya_w4_creds.h"

#define MDNS_PORT        5353
#define MDNS_GROUP       "224.0.0.251"
#define MDNS_TTL_HOST    120u     /* A, SRV, TXT */
#define MDNS_TTL_PTR     4500u    /* service pointers */
#define MDNS_TTL_LEGACY  10u      /* answers to a one-shot unicast query */

#define T_A    1u
#define T_PTR  12u
#define T_TXT  16u
#define T_SRV  33u
#define T_ANY  255u

static const char k_service[]  = "_neuron6._tcp.local";
static const char k_services[] = "_services._dns-sd._udp.local";
static const char k_txt[]      = "board=neuron6";

extern uint32_t MayaW476_GetIPv4_BE(void);

/* Names. */

/* Write a dotted name as DNS labels. Returns bytes written, 0 if no room. */
static size_t put_name(uint8_t *p, size_t room, const char *a, const char *b, const char *c)
{
  const char *parts[3] = { a, b, c };
  size_t n = 0;
  for (int i = 0; i < 3; i++) {
    const char *s = parts[i];
    while (s && *s) {
      const char *dot = strchr(s, '.');
      size_t len = dot ? (size_t)(dot - s) : strlen(s);
      if (len == 0 || len > 63 || n + 1 + len + 1 > room) return 0;
      p[n++] = (uint8_t)len;
      memcpy(&p[n], s, len);
      n += len;
      s = dot ? dot + 1 : NULL;
    }
  }
  if (n + 1 > room) return 0;
  p[n++] = 0;
  return n;
}

/* Read a (possibly compressed) name at `off` into "a.b.c" form. Returns the
 * offset just past it in the packet, or 0 if malformed. */
static size_t get_name(const uint8_t *pkt, size_t len, size_t off, char *out, size_t cap)
{
  size_t o = 0, next = 0;
  int hops = 0;
  while (off < len) {
    uint8_t l = pkt[off];
    if (l == 0) {
      if (next == 0) next = off + 1;
      out[o] = '\0';
      return next;
    }
    if ((l & 0xC0u) == 0xC0u) {
      if (off + 1 >= len || ++hops > 8) return 0;
      if (next == 0) next = off + 2;
      off = (size_t)(((l & 0x3Fu) << 8) | pkt[off + 1]);
      continue;
    }
    if (off + 1 + l > len || o + l + 2 > cap) return 0;
    if (o) out[o++] = '.';
    memcpy(&out[o], &pkt[off + 1], l);
    o += l;
    off += 1 + l;
  }
  return 0;
}

static bool same(const char *a, const char *b)
{
  for (; *a && *b; a++, b++) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
  }
  return *a == *b;
}

/* Records. */

typedef struct {
  uint8_t *buf;
  size_t   cap;
  size_t   len;
  uint16_t count;
  uint32_t ttl_cap;   /* 0 = normal TTLs */
  bool     ok;
} out_t;

static void put16(out_t *o, uint16_t v)
{
  if (o->len + 2 > o->cap) { o->ok = false; return; }
  o->buf[o->len++] = (uint8_t)(v >> 8);
  o->buf[o->len++] = (uint8_t)v;
}

static void put32(out_t *o, uint32_t v)
{
  put16(o, (uint16_t)(v >> 16));
  put16(o, (uint16_t)v);
}

static void rr_head(out_t *o, const char *a, const char *b, const char *c,
                    uint16_t type, bool unique, uint32_t ttl)
{
  size_t n = put_name(o->buf + o->len, o->cap - o->len, a, b, c);
  if (n == 0) { o->ok = false; return; }
  o->len += n;
  put16(o, type);
  /* Cache-flush bit on unique records, but never in a legacy answer (RFC 6762
   * 10.2: its resolver would drop the record). Only legacy answers cap TTL. */
  put16(o, (uint16_t)((unique && (o->ttl_cap == 0u)) ? 0x8001u : 0x0001u));
  put32(o, (o->ttl_cap != 0u && ttl > o->ttl_cap) ? o->ttl_cap : ttl);
  o->count++;
}

static void rr_a(out_t *o, const char *host)
{
  rr_head(o, host, "local", NULL, T_A, true, MDNS_TTL_HOST);
  put16(o, 4);
  const uint32_t ip = MayaW476_GetIPv4_BE();   /* network order in memory */
  if (o->len + 4 > o->cap) { o->ok = false; return; }
  memcpy(o->buf + o->len, &ip, 4);
  o->len += 4;
}

static void rr_ptr(out_t *o, const char *owner, const char *host, const char *svc)
{
  rr_head(o, owner, NULL, NULL, T_PTR, false, MDNS_TTL_PTR);
  const size_t at = o->len;
  put16(o, 0);                                 /* rdlength, filled below */
  size_t n = put_name(o->buf + o->len, o->cap - o->len, host, svc, NULL);
  if (n == 0 || !o->ok) { o->ok = false; return; }
  o->len += n;
  o->buf[at] = (uint8_t)(n >> 8);
  o->buf[at + 1] = (uint8_t)n;
}

static void rr_srv(out_t *o, const char *host)
{
  rr_head(o, host, k_service, NULL, T_SRV, true, MDNS_TTL_HOST);
  const size_t at = o->len;
  put16(o, 0);
  put16(o, 0);                                 /* priority */
  put16(o, 0);                                 /* weight */
  put16(o, maya_w4_get_tcp_port());
  size_t n = put_name(o->buf + o->len, o->cap - o->len, host, "local", NULL);
  if (n == 0 || !o->ok) { o->ok = false; return; }
  o->len += n;
  const size_t rd = 6 + n;
  o->buf[at] = (uint8_t)(rd >> 8);
  o->buf[at + 1] = (uint8_t)rd;
}

static void rr_txt(out_t *o, const char *host)
{
  rr_head(o, host, k_service, NULL, T_TXT, true, MDNS_TTL_HOST);
  const size_t tl = strlen(k_txt);
  put16(o, (uint16_t)(1 + tl));
  if (o->len + 1 + tl > o->cap) { o->ok = false; return; }
  o->buf[o->len++] = (uint8_t)tl;
  memcpy(o->buf + o->len, k_txt, tl);
  o->len += tl;
}

/* Socket. */

int maya_w4_mdns_open(void)
{
  int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return -1;
  int on = 1;
  (void)lwip_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(MDNS_PORT);
  if (lwip_bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    lwip_close(s);
    return -1;
  }

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(MDNS_GROUP);
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  (void)lwip_setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
  unsigned char ttl = 255;
  (void)lwip_setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
  /* Send from the link that is up, whichever interface lwIP calls default. */
  struct in_addr ifaddr;
  ifaddr.s_addr = MayaW476_GetIPv4_BE();
  (void)lwip_setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof(ifaddr));
  return s;
}

static void send_to(int s, const uint8_t *pkt, size_t len, const struct sockaddr_in *to)
{
  struct sockaddr_in dst;
  if (to == NULL) {
    memset(&dst, 0, sizeof(dst));
    dst.sin_family      = AF_INET;
    dst.sin_addr.s_addr = inet_addr(MDNS_GROUP);
    dst.sin_port        = htons(MDNS_PORT);
    to = &dst;
  }
  (void)lwip_sendto(s, pkt, len, 0, (const struct sockaddr *)to, sizeof(*to));
}

/* Unsolicited announcement of every record (RFC 6762 section 8.3). */
void maya_w4_mdns_announce(int s)
{
  if (s < 0 || MayaW476_GetIPv4_BE() == 0U) return;
  static uint8_t pkt[512];
  const char *host = maya_w4_get_hostname();
  out_t o = { pkt, sizeof(pkt), 12, 0, 0, true };
  rr_a(&o, host);
  rr_ptr(&o, k_service, host, k_service);
  rr_srv(&o, host);
  rr_txt(&o, host);
  rr_ptr(&o, k_services, k_service, NULL);
  if (!o.ok) return;
  memset(pkt, 0, 12);
  pkt[2] = 0x84;                               /* response, authoritative */
  pkt[7] = (uint8_t)o.count;                   /* answers */
  send_to(s, pkt, o.len, NULL);
}

/* Read one datagram and answer whatever it asks about the board. */
void maya_w4_mdns_answer(int s)
{
  static uint8_t in[512];
  static uint8_t pkt[512];
  struct sockaddr_in from;
  socklen_t from_len = sizeof(from);
  const int n = lwip_recvfrom(s, in, sizeof(in), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
  if (n < 12 || MayaW476_GetIPv4_BE() == 0U) return;
  if (!maya_w4_wifi_on_link(from.sin_addr.s_addr)) return;   /* RFC 6762, 5.5 */
  if (in[2] & 0x80u) return;                   /* a response, not a query */

  const uint16_t qd = (uint16_t)((in[4] << 8) | in[5]);
  const bool legacy = ntohs(from.sin_port) != MDNS_PORT;   /* one-shot querier */
  const char *host = maya_w4_get_hostname();
  char hostlocal[48], instance[80];
  snprintf(hostlocal, sizeof(hostlocal), "%s.local", host);
  snprintf(instance, sizeof(instance), "%s.%s", host, k_service);

  bool want_a = false, want_ptr = false, want_srv = false, want_txt = false, want_enum = false;
  bool unicast = legacy;
  size_t off = 12, qstart = 12, qend = 12;
  for (uint16_t q = 0; q < qd; q++) {
    char name[128];
    size_t next = get_name(in, (size_t)n, off, name, sizeof(name));
    if (next == 0 || next + 4 > (size_t)n) return;
    const uint16_t type = (uint16_t)((in[next] << 8) | in[next + 1]);
    const uint16_t cls  = (uint16_t)((in[next + 2] << 8) | in[next + 3]);
    off = next + 4;
    if (q == 0) qend = off;
    if (cls & 0x8000u) unicast = true;          /* QU: answer the asker directly */
    const bool any = (type == T_ANY);
    if (same(name, hostlocal) && (any || type == T_A))                 want_a = true;
    if (same(name, k_service) && (any || type == T_PTR))               want_ptr = true;
    if (same(name, instance) && (any || type == T_SRV))                want_srv = true;
    if (same(name, instance) && (any || type == T_TXT))                want_txt = true;
    if (same(name, k_services) && (any || type == T_PTR))              want_enum = true;
  }
  if (!(want_a || want_ptr || want_srv || want_txt || want_enum)) return;

  out_t o = { pkt, sizeof(pkt), 12, 0, legacy ? MDNS_TTL_LEGACY : 0u, true };
  memset(pkt, 0, 12);
  if (legacy) {
    /* A one-shot querier expects its ID and its first question back. */
    const size_t ql = qend - qstart;
    if (12 + ql > sizeof(pkt)) return;
    memcpy(pkt + 12, in + qstart, ql);
    o.len += ql;
    pkt[0] = in[0];
    pkt[1] = in[1];
    pkt[5] = 1;
  }
  if (want_a)    rr_a(&o, host);
  if (want_ptr)  rr_ptr(&o, k_service, host, k_service);
  if (want_srv || want_ptr) rr_srv(&o, host);
  if (want_txt || want_ptr) rr_txt(&o, host);
  if (want_ptr || want_srv) { if (!want_a) rr_a(&o, host); }
  if (want_enum) rr_ptr(&o, k_services, k_service, NULL);
  if (!o.ok || o.count == 0) return;

  pkt[2] = 0x84;
  pkt[6] = (uint8_t)(o.count >> 8);
  pkt[7] = (uint8_t)o.count;
  send_to(s, pkt, o.len, unicast ? &from : NULL);
}

void maya_w4_mdns_close(int s)
{
  if (s >= 0) lwip_close(s);
}
