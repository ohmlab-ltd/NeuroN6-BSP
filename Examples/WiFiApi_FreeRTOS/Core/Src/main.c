/*
  WiFiApi_FreeRTOS

  A command shell over the USB console for the whole Wi-Fi API: joining and
  hosting networks, addressing, scanning, radio off and power save, name
  lookup and network time, the settings a host stored on the board, the link
  key, and lwIP's sockets for TCP and UDP. One command per line; every answer
  line starts "[api]", and each command ends with "[api] ok <cmd>" or
  "[api] fail <cmd> <why>".

    ap SSID KEY [CH [SEC]]    host a network (KEY "-" for the board's own password)
    sta SSID KEY [MS [SEC]]   join one, waiting MS (default 20000; KEY "-" for none)
                              SEC: wpa2 (default), wpa3, or wpa2-wpa3 (WPA3
                              with WPA2 beside it); open, wep, wpa and
                              wpa-wpa2 exist to test that they are refused
    cfg IP [DNS [GW [MASK]]]  a fixed address (0.0.0.0 goes back to DHCP)
    dns IP [IP2]              DNS servers
    host NAME                 the DHCP and mDNS name ("-" for the default)
    stat                      status, addressing, signal, MACs
    scan                      the networks in range
    link                      radio firmware, transmit power, averaged signal
    rate                      the rate the link last ran at, and how
    counters                  frames, retries, failures, beacons
    clients                   the stations joined to the access point
    hidden 1|0                keep the SSID out of the beacon (before ap)
    wake set 1|0              drive the module's wake-in line
    wake [S]                  watch the module's wake-out line for S seconds
    disc                      leave the network, or stop hosting it
    end                       and turn the radio off
    lp 1|0                    IEEE power save on or off
    resolve NAME              name lookup
    tcp HOST PORT N           connect, send N bytes, read the N echoed back
    get HOST PORT PATH        HTTP/1.0 GET: status line and body length
    srv PORT                  a TCP echo server
    udp PORT                  echo every datagram back to its sender
    usend HOST PORT TEXT      send one datagram
    mcast GROUP PORT          report datagrams sent to a multicast group
    time SERVER               network time from that server
    stored                    the stored settings, and the hostname in use
    sbegin                    join or host the stored network
    asset                     what the asset store holds
    awrite TEXT               store TEXT as the user asset (the board restarts)
    key TEXT                  the link key ("-" for none)
    aclear                    remove the user asset (the board restarts)
    pass                      the board's own password (storing it restarts the board)
    pclear                    remove the board's own password (the board restarts)
    wstart MODE               MayaW476_Start: 1 the board's access point, 2 station
    reboot                    restart the board
    bt                        BLE on the same module: a reset and its version
    btinfo                    its address, versions, buffers and limits
    btscan [S [1|0 [MS]]]     listen S seconds for advertisements (0 passive),
                              stalling MS before each read; then the link's health
    btrestart                 advertising and scanning each started twice in a row,
                              and a command allowed HAL_MAX_DELAY
    btadv HEX [MS] | off      advertise HEX every MS
    btconn [S]                advertise connectably, print HCI events for S s
    btpower [S]               BLE transmit power read back from the controller
                              (advertising, extended advertising on each PHY,
                              a 20 dBm request, a connection's levels if a
                              peer connects within S s)
    b                         back to the bootloader

  harness/bench/n6wifiapi.py drives it. Credentials arrive as commands, so
  the example holds none; the only key it prints is the board's own
  password, when asked (pass).

  The circuit:
  - Uses the MAYA-W476 radio module. No wiring needed.
  - The radio's firmware must already be in external flash at 0x91E00000.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_bt.h"
#include "neuron6_com.h"
#include "neuron6_wdg.h"
#include "neuron6_wifi.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "MayaW476.h"
#include "NeuroN6_FrameQueue.h"
#include "n6_host_contract.h"
#include "n6_asset.h"
#include "n6_asset_host.h"

#include "lwip/sockets.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

#define WDG_TIMEOUT_MS            2000U
#define WDG_WARN_MS               300U
#define USB_BOOT_PUMP_MS          500U
#define BEAT_MS                   5000U

#define APP_TASK_STACK_WORDS      ((16U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (4096U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)

#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* The library's power-save bits (1 IEEE power save, 2 deep sleep), for stat. */
extern unsigned maya_w4_wifi_ps_bits(void);

static void mpu_config(void);

/* Shell */

static char     Line[256];
static uint32_t LineLen;

/* The sockets the loop services. */
static int SrvFd = -1, SrvClient = -1, UdpFd = -1, McFd = -1;

static void say(const char *pFmt, ...)
{
  char    buf[240];
  va_list ap;

  va_start(ap, pFmt);
  (void)vsnprintf(buf, sizeof(buf), pFmt, ap);
  va_end(ap);
  print("[api] %s\r\n", buf);
}

static const char *ip_str(uint32_t Ip, char *pOut)
{
  (void)snprintf(pOut, 16U, "%u.%u.%u.%u", (unsigned)(Ip & 0xFFU), (unsigned)((Ip >> 8) & 0xFFU),
                 (unsigned)((Ip >> 16) & 0xFFU), (unsigned)(Ip >> 24));
  return pOut;
}

static const char *mac_str(const uint8_t *pMac, char *pOut)
{
  (void)snprintf(pOut, 18U, "%02X:%02X:%02X:%02X:%02X:%02X",
                 pMac[0], pMac[1], pMac[2], pMac[3], pMac[4], pMac[5]);
  return pOut;
}

static bool ip_arg(const char *pText, uint32_t *pIp)
{
  ip4_addr_t a;

  if ((pText == NULL) || (ip4addr_aton(pText, &a) == 0))
  {
    return false;
  }
  *pIp = ip4_addr_get_u32(&a);
  return true;
}

/* A network's security, as the shell names it. */
static const struct
{
  const char   *pName;
  WifiSecurity Security;
} Securities[] = {
  { "wpa2", WIFI_SECURITY_WPA2 },
  { "wpa3", WIFI_SECURITY_WPA3_SAE },
  { "wpa2-wpa3", WIFI_SECURITY_WPA2_WPA3 },
  /* Refused by MayaW476, named so that the refusals can be tested. */
  { "open", WIFI_SECURITY_NONE },
  { "wep", WIFI_SECURITY_WEP },
  { "wpa", WIFI_SECURITY_WPA },
  { "wpa-wpa2", WIFI_SECURITY_WPA_WPA2 },
};

static bool sec_arg(const char *pText, WifiSecurity *pSec)
{
  for (size_t i = 0U; i < (sizeof(Securities) / sizeof(Securities[0])); i++)
  {
    if (strcmp(pText, Securities[i].pName) == 0)
    {
      *pSec = Securities[i].Security;
      return true;
    }
  }
  return false;
}

static const char *sec_name(WifiSecurity Sec)
{
  for (size_t i = 0U; i < (sizeof(Securities) / sizeof(Securities[0])); i++)
  {
    if (Securities[i].Security == Sec)
    {
      return Securities[i].pName;
    }
  }
  return "?";
}

static void set_addr(struct sockaddr_in *pAddr, uint32_t Ip, uint16_t Port)
{
  memset(pAddr, 0, sizeof(*pAddr));
  pAddr->sin_family      = AF_INET;
  pAddr->sin_port        = lwip_htons(Port);
  pAddr->sin_addr.s_addr = Ip;
}

/* A socket that never blocks, or -1. */
static int open_socket(int Type)
{
  const int fd = lwip_socket(AF_INET, Type, (Type == SOCK_STREAM) ? IPPROTO_TCP : IPPROTO_UDP);
  int       on = 1;

  if (fd >= 0)
  {
    (void)lwip_ioctl(fd, FIONBIO, &on);
  }
  return fd;
}

/* A UDP socket bound to Port, for sending or receiving. */
static int udp_socket(uint16_t Port)
{
  struct sockaddr_in addr;
  const int          fd = open_socket(SOCK_DGRAM);
  int                on = 1;

  if (fd < 0)
  {
    return -1;
  }
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  set_addr(&addr, IPADDR_ANY, Port);
  if (lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
  {
    (void)lwip_close(fd);
    return -1;
  }
  return fd;
}

static void close_socket(int *pFd)
{
  if (*pFd >= 0)
  {
    (void)lwip_close(*pFd);
    *pFd = -1;
  }
}

/* A TCP connection to Host:Port within 10 s, or -1. */
static int tcp_connect(const char *pHost, uint16_t Port)
{
  struct sockaddr_in addr;
  struct timeval     wait = {10, 0};
  fd_set             wr;
  uint32_t           ip  = 0U;
  int                err = -1;
  socklen_t          len = sizeof(err);
  int                on  = 1;
  int                fd;

  if (MayaW476_HostByName(pHost, &ip) == 0)
  {
    return -1;
  }
  fd = open_socket(SOCK_STREAM);
  if (fd < 0)
  {
    return -1;
  }
  set_addr(&addr, ip, Port);
  (void)lwip_connect(fd, (struct sockaddr *)&addr, sizeof(addr));

  /* The outcome arrives as writability; SO_ERROR says which. */
  FD_ZERO(&wr);
  FD_SET(fd, &wr);
  if ((lwip_select(fd + 1, NULL, &wr, NULL, &wait) <= 0) ||
      (lwip_getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0) || (err != 0))
  {
    (void)lwip_close(fd);
    return -1;
  }
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  return fd;
}

/* Send it all, waiting up to 10 s whenever the connection is full. lwIP
 * reports a reset once (here to the send), then SO_ERROR reads 0 and select()
 * says writable; only getpeername() tells a reset socket from a full one. */
static bool send_all(int Fd, const void *pData, uint32_t Len)
{
  const uint8_t *p = (const uint8_t *)pData;

  while (Len > 0U)
  {
    const int          n    = lwip_send(Fd, p, Len, MSG_DONTWAIT);
    struct timeval     wait = {10, 0};
    fd_set             wr;
    int                err  = 0;
    socklen_t          len  = sizeof(err);
    struct sockaddr_in peer;
    socklen_t          plen = sizeof(peer);

    if (n > 0)
    {
      p   += n;
      Len -= (uint32_t)n;
      continue;
    }
    /* Nothing taken: full, or broken. */
    if ((lwip_getsockopt(Fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0) || (err != 0) ||
        (lwip_getpeername(Fd, (struct sockaddr *)&peer, &plen) != 0))
    {
      return false;
    }
    FD_ZERO(&wr);
    FD_SET(Fd, &wr);
    if (lwip_select(Fd + 1, NULL, &wr, NULL, &wait) <= 0)
    {
      return false;
    }
  }
  return true;
}

/* Connect, send N pattern bytes and read the N echoed back, reading while
 * sending so neither end stalls. */
static void tcp_echo(const char *pHost, uint16_t Port, int32_t N)
{
  static uint8_t buf[4096];
  const uint32_t t0   = HAL_GetTick();
  int32_t        sent = 0, got = 0, bad = 0;
  uint32_t       t1;
  int            fd   = tcp_connect(pHost, Port);

  if (fd < 0)
  {
    say("fail tcp connect");
    return;
  }
  t1 = HAL_GetTick();
  while ((got < N) && ((HAL_GetTick() - t1) < 30000U))
  {
    bool moved = false;
    int  n;

    if (sent < N)
    {
      const int32_t k = ((N - sent) < (int32_t)sizeof(buf)) ? (N - sent) : (int32_t)sizeof(buf);

      for (int32_t i = 0; i < k; i++)
      {
        buf[i] = (uint8_t)(((sent + i) * 7) + 3);
      }
      n = lwip_send(fd, buf, (size_t)k, MSG_DONTWAIT);
      if (n > 0)
      {
        sent += n;
        moved = true;
      }
    }
    n = lwip_recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
    if (n == 0)
    {
      break;   /* the other end closed */
    }
    for (int i = 0; i < n; i++)
    {
      bad += (buf[i] != (uint8_t)(((got + i) * 7) + 3)) ? 1 : 0;
    }
    if (n > 0)
    {
      got  += n;
      moved = true;
    }
    if (!moved)
    {
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  }
  (void)lwip_close(fd);
  say("tcp sent=%ld got=%ld bad=%ld connect_ms=%lu total_ms=%lu remote=%s", (long)sent,
      (long)got, (long)bad, (unsigned long)(t1 - t0), (unsigned long)(HAL_GetTick() - t0), pHost);
  if ((sent == N) && (got == N) && (bad == 0))
  {
    say("ok tcp");
  }
  else
  {
    say("fail tcp mismatch");
  }
}

/* An HTTP/1.0 GET: the status line, and how many bytes of body followed. */
static void http_get(const char *pHost, uint16_t Port, const char *pPath)
{
  char     req[192];
  char     status[64] = "";
  char     head[128];
  uint8_t  in[512];
  uint32_t head_len = 0U;
  long     body     = 0;
  bool     in_body  = false;
  bool     first    = true;
  uint32_t t0;
  int      fd       = tcp_connect(pHost, Port);

  if (fd < 0)
  {
    say("fail get connect");
    return;
  }
  (void)snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
                 pPath, pHost);
  if (send_all(fd, req, strlen(req)))
  {
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000U)
    {
      struct timeval wait = {0, 100000};
      fd_set         rd;
      int            n;

      FD_ZERO(&rd);
      FD_SET(fd, &rd);
      if (lwip_select(fd + 1, &rd, NULL, NULL, &wait) <= 0)
      {
        continue;
      }
      n = lwip_recv(fd, in, sizeof(in), MSG_DONTWAIT);
      if (n <= 0)
      {
        break;   /* closed: the whole reply is in */
      }
      for (int i = 0; i < n; i++)
      {
        if (in_body)
        {
          body++;
        }
        else if (in[i] == '\n')
        {
          head[head_len] = '\0';
          if (first)
          {
            (void)snprintf(status, sizeof(status), "%.*s", (int)(sizeof(status) - 1U), head);
            first = false;
          }
          in_body  = (head_len == 0U) || ((head_len == 1U) && (head[0] == '\r'));
          head_len = 0U;
        }
        else if (head_len < (sizeof(head) - 1U))
        {
          head[head_len++] = (char)in[i];
        }
      }
    }
  }
  (void)lwip_close(fd);
  status[strcspn(status, "\r")] = '\0';
  say("get status=\"%s\" body=%ld", status, body);
  if (strncmp(status, "HTTP/1.", 7U) == 0)
  {
    say("ok get");
  }
  else
  {
    say("fail get");
  }
}

/**
  * @brief  Render bytes as hex into a caller's buffer.
  */
static const char *hex_str(const uint8_t *p, uint32_t len, char *out)
{
  static const char k_digits[] = "0123456789ABCDEF";
  uint32_t          i;

  for (i = 0U; (i < len) && (i < BSP_BT_ADV_DATA_MAX); i++)
  {
    out[i * 2U]      = k_digits[p[i] >> 4];
    out[(i * 2U) + 1U] = k_digits[p[i] & 0x0FU];
  }
  out[i * 2U] = '\0';

  return out;
}

/**
  * @brief  Parse a hex string into bytes.
  * @retval Bytes written.
  */
static uint8_t hex_arg(const char *pText, uint8_t *pOut, uint8_t Max)
{
  uint8_t n = 0U;

  while ((pText[0] != '\0') && (pText[1] != '\0') && (n < Max))
  {
    unsigned int byte = 0U;

    if (sscanf(pText, "%2x", &byte) != 1)
    {
      break;
    }
    pOut[n++] = (uint8_t)byte;
    pText    += 2;
  }

  return n;
}

static void stat(void)
{
  uint8_t m1[6], m2[6];
  char    ip[16], mask[16], gw[16], dns[16], bssid[18], mac[18];

  say("stat status=%d ip=%s mask=%s gw=%s dns=%s ssid=%s rssi=%ld ch=%u bssid=%s mac=%s host=%s ps=%u",
      (int)MayaW476_Status(), ip_str(MayaW476_LocalIP(), ip), ip_str(MayaW476_SubnetMask(), mask),
      ip_str(MayaW476_GatewayIP(), gw), ip_str(MayaW476_DnsIP(0), dns), MayaW476_SSID(),
      (long)MayaW476_RSSI(), (unsigned)MayaW476_Channel(), mac_str(MayaW476_BSSID(m1), bssid),
      mac_str(MayaW476_MacAddress(m2), mac), MayaW476_GetHostname(), maya_w4_wifi_ps_bits());
}

static void asset(void)
{
  static uint8_t buf[N6_ASSET_MAX_BYTES];
  char           text[65];
  uint32_t       len = 0U, ver = 0U, wlen = 0U, plen = 0U, i = 0U;
  bool           user = n6_asset_present(&ver, &len) && n6_asset_read(buf, sizeof(buf), &len, &ver);
  const bool     wifi = n6_asset_record_read(N6_ASSET_REC_WIFI, NULL, 0U, &wlen);
  const bool     pass = n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, NULL, 0U, &plen);

  for (; user && (i < len) && (i < (sizeof(text) - 1U)); i++)
  {
    text[i] = ((buf[i] >= 32U) && (buf[i] < 127U)) ? (char)buf[i] : '.';
  }
  text[i] = '\0';
  say("asset user=%s len=%lu ver=%lu text=\"%s\" wifi=%s wlen=%lu appass=%s fsbl=\"%s\"",
      user ? "yes" : "no", (unsigned long)(user ? len : 0U), (unsigned long)(user ? ver : 0U),
      text, wifi ? "yes" : "no", (unsigned long)wlen, pass ? "yes" : "no",
      n6_asset_result_str(n6_asset_last_result()));
}

static void handle(char *pCmd)
{
  char       *argv[8];
  int         argc = 0;
  const char *c;
  char        ip[16];
  char        hexbuf[(BSP_BT_ADV_DATA_MAX * 2U) + 1U];
  uint32_t    t0;

  for (char *t = strtok(pCmd, " "); (t != NULL) && (argc < 8); t = strtok(NULL, " "))
  {
    argv[argc++] = t;
  }
  if (argc == 0)
  {
    return;
  }
  c = argv[0];

  if ((strcmp(c, "ap") == 0) && (argc >= 3))
  {
    const char  *key = (strcmp(argv[2], "-") != 0) ? argv[2] : NULL;   /* NULL: the board's own */
    WifiSecurity sec = WIFI_SECURITY_WPA2;
    wl_status_t  s;

    if ((argc >= 5) && !sec_arg(argv[4], &sec))
    {
      say("fail ap security");
      return;
    }
    s = MayaW476_BeginAP(argv[1], key, (argc >= 4) ? (uint8_t)atoi(argv[3]) : 6U, sec);
    say("ap status=%d ip=%s sec=%s", (int)s, ip_str(MayaW476_LocalIP(), ip), sec_name(sec));
    say(((s == WL_AP_LISTENING) || (s == WL_AP_CONNECTED)) ? "ok ap" : "fail ap");
  }
  else if ((strcmp(c, "sta") == 0) && (argc >= 3))
  {
    const char  *key = (strcmp(argv[2], "-") != 0) ? argv[2] : NULL;
    WifiSecurity sec = WIFI_SECURITY_WPA2;
    wl_status_t  s;

    if ((argc >= 5) && !sec_arg(argv[4], &sec))
    {
      say("fail sta security");
      return;
    }
    MayaW476_SetTimeout((argc >= 4) ? (uint32_t)atol(argv[3]) : 20000U);
    s = MayaW476_Begin(argv[1], key, sec);
    say("sta status=%d ip=%s sec=%s", (int)s, ip_str(MayaW476_LocalIP(), ip), sec_name(sec));
    say((s == WL_CONNECTED) ? "ok sta" : "fail sta");
  }
  else if ((strcmp(c, "cfg") == 0) && (argc >= 2))
  {
    uint32_t a = 0U, dns = 0U, gw = 0U, mask = 0U;

    if (!ip_arg(argv[1], &a))
    {
      say("fail cfg ip");
      return;
    }
    /* Omitted fields take the library's defaults (MayaW476_Config). */
    if (argc >= 3) { (void)ip_arg(argv[2], &dns); }
    if (argc >= 4) { (void)ip_arg(argv[3], &gw); }
    if (argc >= 5) { (void)ip_arg(argv[4], &mask); }
    MayaW476_Config(a, dns, gw, mask);
    say("ok cfg");
  }
  else if ((strcmp(c, "dns") == 0) && (argc >= 2))
  {
    uint32_t a = 0U, b = 0U;

    if (!ip_arg(argv[1], &a))
    {
      say("fail dns");
      return;
    }
    if (argc >= 3)
    {
      (void)ip_arg(argv[2], &b);
    }
    MayaW476_SetDNS(a, b);
    say("ok dns");
  }
  else if ((strcmp(c, "host") == 0) && (argc >= 2))
  {
    MayaW476_SetHostname((strcmp(argv[1], "-") != 0) ? argv[1] : "");   /* "-": the default */
    say("host name=%s", MayaW476_GetHostname());
    say("ok host");
  }
  else if (strcmp(c, "stat") == 0)
  {
    stat();
    say("ok stat");
  }
  else if (strcmp(c, "scan") == 0)
  {
    int8_t n;

    t0 = HAL_GetTick();
    n  = MayaW476_ScanNetworks();
    for (int i = 0; i < n; i++)
    {
      uint8_t m[6];
      char    b[18];

      say("net %d ssid=\"%s\" rssi=%ld enc=%u ch=%u bssid=%s", i, MayaW476_ScanSSID((uint8_t)i),
          (long)MayaW476_ScanRSSI((uint8_t)i), (unsigned)MayaW476_ScanEncryption((uint8_t)i),
          (unsigned)MayaW476_ScanChannel((uint8_t)i), mac_str(MayaW476_ScanBSSID((uint8_t)i, m), b));
    }
    say("scan n=%d ms=%lu status=%d", (int)n, (unsigned long)(HAL_GetTick() - t0), (int)MayaW476_Status());
    say((n >= 0) ? "ok scan" : "fail scan");
  }
  else if (strcmp(c, "disc") == 0)
  {
    MayaW476_Disconnect();
    say("disc status=%d", (int)MayaW476_Status());
    say("ok disc");
  }
  else if (strcmp(c, "end") == 0)
  {
    MayaW476_End();
    say("end status=%d", (int)MayaW476_Status());
    say("ok end");
  }
  else if ((strcmp(c, "lp") == 0) && (argc >= 2))
  {
    const bool r = (atoi(argv[1]) != 0) ? MayaW476_LowPowerMode() : MayaW476_NoLowPowerMode();

    say(r ? "ok lp" : "fail lp");
  }
  else if (strcmp(c, "link") == 0)
  {
    int32_t rssi = 0;
    int32_t snr  = 0;
    const bool ok = MayaW476_AverageSignal(&rssi, &snr);

    say("link fw=\"%s\" txpower=%ld avg=%d rssi=%ld snr=%ld inst=%ld",
        MayaW476_FirmwareVersion(), (long)MayaW476_TxPower(), (int)ok,
        (long)rssi, (long)snr, (long)MayaW476_RSSI());
    say("ok link");
  }
  else if (strcmp(c, "rate") == 0)
  {
    MayaW476_PhyRate tx, rx;

    if (MayaW476_DataRate(&tx, &rx))
    {
      say("rate tx=%lu rx=%lu txmcs=%u rxmcs=%u txbw=%u rxbw=%u txsgi=%u "
          "rxsgi=%u txfmt=%u rxfmt=%u",
          (unsigned long)tx.kbit_s, (unsigned long)rx.kbit_s,
          (unsigned)tx.mcs, (unsigned)rx.mcs, (unsigned)tx.bandwidth,
          (unsigned)rx.bandwidth, (unsigned)tx.short_gi, (unsigned)rx.short_gi,
          (unsigned)tx.format, (unsigned)rx.format);
      say("ok rate");
    }
    else
    {
      say("fail rate");
    }
  }
  else if (strcmp(c, "counters") == 0)
  {
    MayaW476_Counters ct;

    if (MayaW476_GetCounters(&ct))
    {
      say("counters tx=%lu failed=%lu retry=%lu multi=%lu ackfail=%lu "
          "fcs=%lu mcrx=%lu bcn=%lu bcnmiss=%lu",
          (unsigned long)ct.tx_frames, (unsigned long)ct.tx_failed,
          (unsigned long)ct.tx_retry, (unsigned long)ct.tx_multi_retry,
          (unsigned long)ct.ack_failure, (unsigned long)ct.fcs_error,
          (unsigned long)ct.rx_multicast, (unsigned long)ct.beacon_rx,
          (unsigned long)ct.beacon_miss);
      say("ok counters");
    }
    else
    {
      say("fail counters");
    }
  }
  else if (strcmp(c, "hidden") == 0)
  {
    /* Takes effect at the next ap command, not on a running access point. */
    if (argc >= 2)
    {
      MayaW476_SetApHidden(atoi(argv[1]) != 0);
    }
    say("hidden %d", (int)MayaW476_ApHidden());
    say("ok hidden");
  }
  else if (strcmp(c, "clients") == 0)
  {
    MayaW476_Client cl[8];
    const uint8_t   n = MayaW476_Clients(cl, (uint8_t)(sizeof(cl) / sizeof(cl[0])));
    uint8_t         i;

    say("clients n=%u", (unsigned)n);
    for (i = 0U; (i < n) && (i < (uint8_t)(sizeof(cl) / sizeof(cl[0]))); i++)
    {
      say("client %u mac=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d ps=%u",
          (unsigned)i, cl[i].mac[0], cl[i].mac[1], cl[i].mac[2],
          cl[i].mac[3], cl[i].mac[4], cl[i].mac[5],
          (int)cl[i].rssi, (unsigned)cl[i].power_save);
    }
    say("ok clients");
  }
  else if (strcmp(c, "wake") == 0)
  {
    /* "wake set" drives the host-to-module line; "wake" samples the
     * module-to-host line. */
    if ((argc >= 3) && (strcmp(argv[1], "set") == 0))
    {
      const int32_t r = BSP_WIFI_SetWake((uint8_t)(atoi(argv[2]) != 0));

      say("wake set=%d status=%ld out=%u", (atoi(argv[2]) != 0), (long)r,
          (unsigned)BSP_WIFI_GetWakeOut());
      say((r == BSP_ERROR_NONE) ? "ok wake" : "fail wake");
    }
    else
    {
      /* Sample as fast as possible and count edges: the module pulses this
       * line, so an occasional read would miss it. */
      const uint32_t secs  = (argc >= 2) ? (uint32_t)atol(argv[1]) : 1U;
      uint32_t       highs = 0U;
      uint32_t       edges = 0U;
      uint32_t       first = 0U;
      uint32_t       reads = 0U;
      uint8_t        last  = BSP_WIFI_GetWakeOut();

      t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < (secs * 1000U))
      {
        const uint8_t now = BSP_WIFI_GetWakeOut();

        reads++;
        if (now != 0U)
        {
          highs++;
        }
        if (now != last)
        {
          edges++;
          if ((first == 0U) && (now != 0U))
          {
            first = HAL_GetTick() - t0;
          }
          last = now;
        }
        /* Yield now and then so the watchdog and the console still run. */
        if ((reads & 0x3FFFU) == 0U)
        {
          vTaskDelay(1);
        }
      }
      say("wake out=%u reads=%lu high=%lu edges=%lu first=%lums secs=%lu",
          (unsigned)BSP_WIFI_GetWakeOut(), (unsigned long)reads,
          (unsigned long)highs, (unsigned long)edges, (unsigned long)first,
          (unsigned long)secs);
      say("ok wake");
    }
  }
  else if ((strcmp(c, "resolve") == 0) && (argc >= 2))
  {
    uint32_t a = 0U;
    int      r;

    t0 = HAL_GetTick();
    r  = MayaW476_HostByName(argv[1], &a);
    say("resolve %s -> %s (%lu ms)", argv[1], ip_str(a, ip), (unsigned long)(HAL_GetTick() - t0));
    say((r != 0) ? "ok resolve" : "fail resolve");
  }
  else if ((strcmp(c, "tcp") == 0) && (argc >= 4))
  {
    tcp_echo(argv[1], (uint16_t)atoi(argv[2]), (int32_t)atol(argv[3]));
  }
  else if ((strcmp(c, "get") == 0) && (argc >= 4))
  {
    http_get(argv[1], (uint16_t)atoi(argv[2]), argv[3]);
  }
  else if ((strcmp(c, "srv") == 0) && (argc >= 2))
  {
    struct sockaddr_in addr;
    int                on = 1;

    close_socket(&SrvClient);
    close_socket(&SrvFd);
    SrvFd = open_socket(SOCK_STREAM);
    set_addr(&addr, IPADDR_ANY, (uint16_t)atoi(argv[1]));
    if ((SrvFd >= 0) &&
        ((lwip_setsockopt(SrvFd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0) ||
         (lwip_bind(SrvFd, (struct sockaddr *)&addr, sizeof(addr)) != 0) ||
         (lwip_listen(SrvFd, 2) != 0)))
    {
      close_socket(&SrvFd);
    }
    say((SrvFd >= 0) ? "ok srv" : "fail srv");
  }
  else if ((strcmp(c, "udp") == 0) && (argc >= 2))
  {
    close_socket(&UdpFd);
    UdpFd = udp_socket((uint16_t)atoi(argv[1]));
    say((UdpFd >= 0) ? "ok udp" : "fail udp");
  }
  else if ((strcmp(c, "usend") == 0) && (argc >= 4))
  {
    struct sockaddr_in to;
    uint32_t           a  = 0U;
    int                fd = udp_socket(0U);
    const size_t       n  = strlen(argv[3]);
    bool               r  = (fd >= 0) && (MayaW476_HostByName(argv[1], &a) != 0);

    if (r)
    {
      set_addr(&to, a, (uint16_t)atoi(argv[2]));
      r = (lwip_sendto(fd, argv[3], n, 0, (struct sockaddr *)&to, sizeof(to)) == (int)n);
    }
    close_socket(&fd);
    say(r ? "ok usend" : "fail usend");
  }
  else if ((strcmp(c, "mcast") == 0) && (argc >= 3))
  {
    struct ip_mreq mreq;
    uint32_t       group = 0U;

    close_socket(&McFd);
    if (ip_arg(argv[1], &group))
    {
      McFd = udp_socket((uint16_t)atoi(argv[2]));
      mreq.imr_multiaddr.s_addr = group;
      mreq.imr_interface.s_addr = IPADDR_ANY;
      if ((McFd >= 0) &&
          (lwip_setsockopt(McFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0))
      {
        close_socket(&McFd);
      }
    }
    say((McFd >= 0) ? "ok mcast" : "fail mcast");
  }
  else if ((strcmp(c, "time") == 0) && (argc >= 2))
  {
    uint32_t t;

    MayaW476_SetTimeServer(argv[1]);
    t0 = HAL_GetTick();
    t  = MayaW476_GetTime();
    say("time epoch=%lu (%lu ms)", (unsigned long)t, (unsigned long)(HAL_GetTick() - t0));
    say((t != 0U) ? "ok time" : "fail time");
  }
  else if ((strcmp(c, "key") == 0) && (argc >= 2))
  {
    say(MayaW476_SetLinkKey((strcmp(argv[1], "-") != 0) ? argv[1] : "") ? "ok key" : "fail key length");
  }
  else if (strcmp(c, "stored") == 0)
  {
    say("stored mode=%d ssid=\"%s\" hostname=%s", (int)MayaW476_StoredMode(), MayaW476_StoredSSID(),
        MayaW476_GetHostname());
    say("ok stored");
  }
  else if (strcmp(c, "sbegin") == 0)
  {
    wl_status_t s;

    MayaW476_SetTimeout(20000U);
    s = MayaW476_BeginStored();
    say("sbegin status=%d ip=%s ssid=\"%s\"", (int)s, ip_str(MayaW476_LocalIP(), ip), MayaW476_SSID());
    say(((s == WL_CONNECTED) || (s == WL_AP_LISTENING) || (s == WL_AP_CONNECTED)) ? "ok sbegin" : "fail sbegin");
  }
  else if (strcmp(c, "asset") == 0)
  {
    asset();
    say("ok asset");
  }
  else if (strcmp(c, "pass") == 0)
  {
    /* The first call stores it and restarts the board: say so first. */
    uint32_t    n = 0U;
    const char *k;

    if (!n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, NULL, 0U, &n))
    {
      say("ok pass storing");
      BSP_COM_Flush(200U);
      vTaskDelay(pdMS_TO_TICKS(200U));
    }
    k = MayaW476_DevicePassword();
    say("pass key=%s len=%u", k, (unsigned)strlen(k));
    say((strlen(k) >= 8U) ? "ok pass" : "fail pass");
  }
  else if ((strcmp(c, "wstart") == 0) && (argc >= 2))
  {
    const int rc = MayaW476_Start(atoi(argv[1]));

    say("wstart rc=%d status=%d", rc, (int)MayaW476_Status());
    say((rc == 0) ? "ok wstart" : "fail wstart");
  }
  else if (strcmp(c, "reboot") == 0)
  {
    say("ok reboot");
    BSP_COM_Flush(200U);
    vTaskDelay(pdMS_TO_TICKS(200U));
    n6_asset_apply();   /* a reset that clears the double-press key, with nothing staged */
  }
  else if (((strcmp(c, "awrite") == 0) && (argc >= 2)) || (strcmp(c, "aclear") == 0) ||
           (strcmp(c, "pclear") == 0))
  {
    const bool ok = (c[1] == 'w') ? n6_asset_stage(argv[1], strlen(argv[1]), 0U)
                  : (c[0] == 'a') ? n6_asset_record_stage(N6_ASSET_REC_USER, NULL, 0U)
                                  : n6_asset_record_stage(N6_ASSET_REC_AP_PASSWORD, NULL, 0U);

    if (!ok)
    {
      say("fail %s staging refused", c);
      return;
    }
    say("ok %s restarting", c);
    BSP_COM_Flush(200U);
    vTaskDelay(pdMS_TO_TICKS(200U));   /* for the host to read it before USB goes */
    n6_asset_apply();                  /* the FSBL writes it on the way back up */
  }
  else if (strcmp(c, "bt") == 0)
  {
    /* BLE on the same module: the firmware (loaded once), the link, then
     * HCI_Reset and HCI_Read_Local_Version. */
    static bool bt_up;
    uint8_t     v[8] = {0};
    int32_t     r    = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x1001U, NULL, 0U, v, sizeof(v), 2000U);
    }
    say("bt status=%ld hci=%u manufacturer=0x%04X wifi=%d", (long)r, (unsigned)v[0],
        (unsigned)v[4] | ((unsigned)v[5] << 8), (int)MayaW476_Status());
    say((r == BSP_ERROR_NONE) ? "ok bt" : "fail bt");
  }
  else if (strcmp(c, "btinfo") == 0)
  {
    static bool   bt_up;
    BSP_BT_Info_t info;
    int32_t       r = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_GetInfo(&info);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* Address most significant byte first (HCI carries it the other way);
       * the revisions are vendor build numbers, printed as fields of their own. */
      say("btinfo addr=%02X:%02X:%02X:%02X:%02X:%02X hci=%u rev=%u lmp=%u sub=%u "
          "manufacturer=0x%04X acl=%ux%u adv=%u sets=%u txpower=%d..%d "
          "features=%02X%02X%02X%02X%02X%02X%02X%02X",
          info.BdAddr[5], info.BdAddr[4], info.BdAddr[3],
          info.BdAddr[2], info.BdAddr[1], info.BdAddr[0],
          (unsigned)info.HciVersion, (unsigned)info.HciRevision,
          (unsigned)info.LmpVersion, (unsigned)info.LmpSubversion,
          (unsigned)info.Manufacturer, (unsigned)info.LeAclPackets,
          (unsigned)info.LeAclDataLength, (unsigned)info.MaxAdvDataLength,
          (unsigned)info.AdvSets, (int)info.TxPowerMin, (int)info.TxPowerMax,
          info.LeFeatures[0], info.LeFeatures[1], info.LeFeatures[2],
          info.LeFeatures[3], info.LeFeatures[4], info.LeFeatures[5],
          info.LeFeatures[6], info.LeFeatures[7]);
    }
    say((r == BSP_ERROR_NONE) ? "ok btinfo" : "fail btinfo");
  }
  else if (strcmp(c, "btscan") == 0)
  {
    static bool            bt_up;
    BSP_BT_Advertisement_t adv;
    const uint32_t         secs = (argc >= 2) ? (uint32_t)atol(argv[1]) : 5U;
    uint32_t               n    = 0U;
    int32_t                r    = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE
                                                               : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    /* Window equal to the interval: listen all the time, to find advertisers fastest. */
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_ScanStart((argc >= 3) ? (uint8_t)(atoi(argv[2]) != 0) : 1U, 60U, 60U);
    }
    if (r == BSP_ERROR_NONE)
    {
      const uint32_t stall = (argc >= 4) ? (uint32_t)atol(argv[3]) : 0U;
      BSP_BT_Health_t health;

      t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < (secs * 1000U))
      {
        if (stall != 0U)
        {
          vTaskDelay(pdMS_TO_TICKS(stall));
        }
        if (BSP_BT_ScanNext(&adv, 500U) != BSP_ERROR_NONE)
        {
          continue;
        }
        n++;
        say("adv addr=%02X:%02X:%02X:%02X:%02X:%02X type=%u evt=%u rssi=%d len=%u data=%s",
            adv.Address[5], adv.Address[4], adv.Address[3], adv.Address[2],
            adv.Address[1], adv.Address[0], (unsigned)adv.AddressType,
            (unsigned)adv.EventType, (int)adv.Rssi, (unsigned)adv.DataLength,
            hex_str(adv.Data, adv.DataLength, hexbuf));
      }
      (void)BSP_BT_ScanStop();
      say("btscan seen=%lu secs=%lu", (unsigned long)n, (unsigned long)secs);
      if (BSP_BT_GetHealth(&health) == BSP_ERROR_NONE)
      {
        say("btscan health bytes=%lu packets=%lu overflows=%lu framing=%lu uart=%lu",
            (unsigned long)health.BytesReceived, (unsigned long)health.PacketsReceived,
            (unsigned long)health.Overflows, (unsigned long)health.FramingErrors,
            (unsigned long)health.UartErrors);
      }
    }
    say((r == BSP_ERROR_NONE) ? "ok btscan" : "fail btscan");
  }
  else if (strcmp(c, "btrestart") == 0)
  {
    /* Advertising and scanning each started twice back to back, as a caller
     * changing its settings would, then a command allowed HAL_MAX_DELAY. */
    static const uint8_t payload_a[] = { 0x02U, 0x01U, 0x06U, 0x03U, 0xFFU, 0xFFU, 0xFFU, 0x41U };
    static const uint8_t payload_b[] = { 0x02U, 0x01U, 0x06U, 0x03U, 0xFFU, 0xFFU, 0xFFU, 0x42U };
    uint8_t version[8] = {0};
    int32_t a1 = -99, a2 = -99, s1 = -99, s2 = -99, forever = -99;
    int32_t r = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Init();
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      a1 = BSP_BT_AdvertiseStart(0U, 100U, payload_a, (uint8_t)sizeof(payload_a));
      a2 = BSP_BT_AdvertiseStart(0U, 100U, payload_b, (uint8_t)sizeof(payload_b));
      (void)BSP_BT_AdvertiseStop();
      s1 = BSP_BT_ScanStart(1U, 60U, 60U);
      s2 = BSP_BT_ScanStart(0U, 60U, 60U);
      (void)BSP_BT_ScanStop();
      (void)BSP_BT_Flush();
      forever = BSP_BT_Command(0x1001U, NULL, 0U, version, sizeof(version), HAL_MAX_DELAY);
    }
    say("btrestart advertise %ld then %ld, scan %ld then %ld, a command with HAL_MAX_DELAY %ld",
        (long)a1, (long)a2, (long)s1, (long)s2, (long)forever);
    say(((r == BSP_ERROR_NONE) && (a1 == 0) && (a2 == 0) && (s1 == 0) && (s2 == 0) && (forever == 0))
        ? "ok btrestart" : "fail btrestart");
  }
  else if (strcmp(c, "btadv") == 0)
  {
    /* The payload is AD structures: a length byte, then that many bytes. */
    static bool bt_up;
    uint8_t     payload[BSP_BT_ADV_DATA_MAX];
    uint8_t     len = 0U;
    int32_t     r   = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    /* The controller answers nothing until it has been reset. */
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if ((r == BSP_ERROR_NONE) && (argc >= 2) && (strcmp(argv[1], "off") == 0))
    {
      r = BSP_BT_AdvertiseStop();
      say("btadv off");
    }
    else if (r == BSP_ERROR_NONE)
    {
      if (argc >= 2)
      {
        len = hex_arg(argv[1], payload, (uint8_t)sizeof(payload));
      }
      r = BSP_BT_AdvertiseStart(0U, (argc >= 3) ? (uint32_t)atol(argv[2]) : 100U,
                                (len > 0U) ? payload : NULL, len);
      say("btadv len=%u", (unsigned)len);
    }
    say((r == BSP_ERROR_NONE) ? "ok btadv" : "fail btadv");
  }
  else if (strcmp(c, "btconn") == 0)
  {
    /* Advertise connectably and print the controller's events. No host stack
     * runs here: this shows the events reach the application. */
    static bool    bt_up;
    uint8_t        pkt[BSP_BT_MAX_PACKET_SIZE];
    const uint32_t secs = (argc >= 2) ? (uint32_t)atol(argv[1]) : 20U;
    uint32_t       events = 0U;
    uint32_t       conns  = 0U;
    int32_t        r = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* Flags, and a name for the scanner to show. */
      static const uint8_t adv[] = { 2U, 0x01U, 0x06U,
                                     6U, 0x09U, 'N', '6', 'C', 'O', 'N' };

      r = BSP_BT_AdvertiseStart(1U, 100U, adv, (uint8_t)sizeof(adv));
    }
    if (r == BSP_ERROR_NONE)
    {
      t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < (secs * 1000U))
      {
        uint32_t len = 0U;

        if (BSP_BT_Receive(pkt, sizeof(pkt), &len, 500U) != BSP_ERROR_NONE)
        {
          continue;
        }
        events++;
        if ((len >= 15U) && (pkt[0] == 0x04U) && (pkt[1] == 0x3EU) && (pkt[3] == 0x01U))
        {
          /* LE connection complete: status at 4, handle at 5, role at 7,
           * the peer's address type at 8 and its address at 9 to 14. */
          conns++;
          say("btconn connected status=%u handle=%u role=%u type=%u "
              "peer=%02X:%02X:%02X:%02X:%02X:%02X",
              (unsigned)pkt[4], (unsigned)(pkt[5] | (pkt[6] << 8)),
              (unsigned)pkt[7], (unsigned)pkt[8],
              pkt[14], pkt[13], pkt[12], pkt[11], pkt[10], pkt[9]);
        }
        else if ((len >= 7U) && (pkt[0] == 0x04U) && (pkt[1] == 0x05U))
        {
          say("btconn disconnected handle=%u reason=0x%02X",
              (unsigned)(pkt[4] | (pkt[5] << 8)), (unsigned)pkt[6]);
        }
        else
        {
          say("btconn event=0x%02X sub=0x%02X len=%lu", (unsigned)pkt[1],
              (unsigned)((len >= 4U) ? pkt[3] : 0U), (unsigned long)len);
        }
      }
      (void)BSP_BT_AdvertiseStop();
      say("btconn events=%lu connections=%lu secs=%lu", (unsigned long)events,
          (unsigned long)conns, (unsigned long)secs);
    }
    say((r == BSP_ERROR_NONE) ? "ok btconn" : "fail btconn");
  }
  else if (strcmp(c, "btpower") == 0)
  {
    /* BLE transmit power read back, against the 8 dBm cap neuron6_bt holds
     * every command to: advertising before and after NXP's power command
     * asking 20 dBm, extended advertising on each PHY pair (no preference and
     * 20 dBm), and a connection's levels if a peer connects within S s. The
     * controller is reset at the end, back to its defaults. */
    static bool          bt_up;
    static const uint8_t phys[3][2] = { { 1U, 1U }, { 1U, 2U }, { 3U, 3U } };
    static const uint8_t asks[2]    = { 0x7FU, 20U };
    uint8_t              pkt[BSP_BT_MAX_PACKET_SIZE];
    uint8_t              ret[8] = { 0U };
    const uint32_t       secs   = (argc >= 2) ? (uint32_t)atol(argv[1]) : 20U;
    uint16_t             handle = 0xFFFFU;
    int32_t              r = (MayaW476_RadioStart() == 0) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;

    if ((r == BSP_ERROR_NONE) && !bt_up)
    {
      r     = BSP_BT_Init();
      bt_up = (r == BSP_ERROR_NONE);
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* LE Read Advertising Physical Channel Tx Power. */
      r = BSP_BT_Command(0x2007U, NULL, 0U, ret, 1U, 1000U);
      say("btpower adv default=%d dBm r=%ld", (int)(int8_t)ret[0], (long)r);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* Level, front-end loss flag, limit. neuron6_bt sends 8, 0, 8. */
      static const uint8_t ask[3] = { 20U, 1U, 20U };
      const int32_t        q      = BSP_BT_Command(0xFC87U, ask, 3U, NULL, 0U, 1000U);

      ret[0] = 0U;
      r = BSP_BT_Command(0x2007U, NULL, 0U, ret, 1U, 1000U);
      say("btpower adv after-fc87-20=%d dBm fc87=%ld r=%ld", (int)(int8_t)ret[0], (long)q, (long)r);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* Extended advertising commands follow the legacy ones only after a
       * reset. */
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    for (uint32_t p = 0U; (r == BSP_ERROR_NONE) && (p < 3U); p++)
    {
      for (uint32_t a = 0U; (r == BSP_ERROR_NONE) && (a < 2U); a++)
      {
        /* LE Set Extended Advertising Parameters: set 0, non-connectable
         * extended advertising every 100 ms on all three channels. */
        uint8_t q[25] = { 0U };

        q[3]  = 0xA0U;          /* primary interval, least and most */
        q[6]  = 0xA0U;
        q[9]  = 0x07U;          /* all three channels */
        q[19] = asks[a];        /* the power asked for */
        q[20] = phys[p][0];     /* primary PHY */
        q[22] = phys[p][1];     /* secondary PHY */
        ret[0] = 0U;
        r = BSP_BT_Command(0x2036U, q, (uint8_t)sizeof(q), ret, 1U, 1000U);
        say("btpower ext primary=%u secondary=%u ask=%d selected=%d dBm r=%ld",
            (unsigned)phys[p][0], (unsigned)phys[p][1], (int)(int8_t)asks[a],
            (int)(int8_t)ret[0], (long)r);
      }
    }
    if (r == BSP_ERROR_NONE)
    {
      r = BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    }
    if (r == BSP_ERROR_NONE)
    {
      /* Read BD_ADDR, for the peer to connect to. */
      r = BSP_BT_Command(0x1009U, NULL, 0U, ret, 6U, 1000U);
      say("btpower addr %02X:%02X:%02X:%02X:%02X:%02X",
          ret[5], ret[4], ret[3], ret[2], ret[1], ret[0]);
    }
    if (r == BSP_ERROR_NONE)
    {
      static const uint8_t adv[] = { 2U, 0x01U, 0x06U,
                                     6U, 0x09U, 'N', '6', 'P', 'W', 'R' };

      r = BSP_BT_AdvertiseStart(1U, 100U, adv, (uint8_t)sizeof(adv));
    }
    if (r == BSP_ERROR_NONE)
    {
      const uint32_t start = HAL_GetTick();

      while ((handle == 0xFFFFU) && ((HAL_GetTick() - start) < (secs * 1000U)))
      {
        uint32_t len = 0U;

        /* LE Connection Complete or LE Enhanced Connection Complete, status 0:
         * the handle at 5. */
        if ((BSP_BT_Receive(pkt, sizeof(pkt), &len, 500U) == BSP_ERROR_NONE) &&
            (len >= 7U) && (pkt[0] == 0x04U) && (pkt[1] == 0x3EU) &&
            ((pkt[3] == 0x01U) || (pkt[3] == 0x0AU)) && (pkt[4] == 0x00U))
        {
          handle = (uint16_t)(pkt[5] | (pkt[6] << 8));
        }
      }
      (void)BSP_BT_AdvertiseStop();
      if (handle == 0xFFFFU)
      {
        say("btpower conn none in %lu s", (unsigned long)secs);
      }
      for (uint8_t type = 0U; (handle != 0xFFFFU) && (type < 2U); type++)
      {
        /* HCI Read Transmit Power Level: 0 current, 1 the most. */
        const uint8_t q[3] = { (uint8_t)handle, (uint8_t)(handle >> 8), type };
        int32_t       s;

        ret[2] = 0U;
        s = BSP_BT_Command(0x0C2DU, q, 3U, ret, 3U, 1000U);
        say("btpower conn handle=%u %s=%d dBm r=%ld", (unsigned)handle,
            (type == 0U) ? "current" : "max", (int)(int8_t)ret[2], (long)s);
      }
      for (uint8_t phy = 1U; (handle != 0xFFFFU) && (phy <= 4U); phy++)
      {
        /* LE Enhanced Read Transmit Power Level: 1M, 2M, Coded S8, Coded S2. */
        const uint8_t q[3] = { (uint8_t)handle, (uint8_t)(handle >> 8), phy };
        int32_t       s;

        ret[3] = 0U;
        ret[4] = 0U;
        s = BSP_BT_Command(0x2076U, q, 3U, ret, 5U, 1000U);
        say("btpower conn phy=%u current=%d max=%d dBm r=%ld", (unsigned)phy,
            (int)(int8_t)ret[3], (int)(int8_t)ret[4], (long)s);
      }
    }
    /* The power command's setting goes with a reset. */
    (void)BSP_BT_Command(0x0C03U, NULL, 0U, NULL, 0U, 2000U);
    say((r == BSP_ERROR_NONE) ? "ok btpower" : "fail btpower");
  }
  else if (strcmp(c, "b") == 0)
  {
    BSP_COM_RebootToBootloader();
  }
  else
  {
    say("fail %s unknown", c);
  }
}

/* The sockets a command left open: echo what arrives. */
static void serve_sockets(void)
{
  uint8_t            buf[1472];
  struct sockaddr_in from;
  socklen_t          from_len = sizeof(from);
  int                n;

  if ((SrvFd >= 0) && (SrvClient < 0))
  {
    SrvClient = lwip_accept(SrvFd, NULL, NULL);
  }
  if (SrvClient >= 0)
  {
    struct sockaddr_in peer;
    socklen_t          plen = sizeof(peer);

    /* With nothing to read, a live connection still has its peer; one the
     * client reset has none (as in send_all()). */
    n = lwip_recv(SrvClient, buf, sizeof(buf), MSG_DONTWAIT);
    if ((n == 0) ||
        ((n < 0) && (lwip_getpeername(SrvClient, (struct sockaddr *)&peer, &plen) != 0)) ||
        ((n > 0) && !send_all(SrvClient, buf, (uint32_t)n)))
    {
      close_socket(&SrvClient);   /* the client left */
    }
  }
  if (UdpFd >= 0)
  {
    n = lwip_recvfrom(UdpFd, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
    if (n > 0)
    {
      (void)lwip_sendto(UdpFd, buf, (size_t)n, 0, (struct sockaddr *)&from, from_len);
    }
  }
  if (McFd >= 0)
  {
    char ip[16];

    from_len = sizeof(from);
    n = lwip_recvfrom(McFd, buf, sizeof(buf) - 1U, MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
    if (n > 0)
    {
      buf[n] = 0U;
      say("mcast from=%s len=%d text=\"%s\"", ip_str(from.sin_addr.s_addr, ip), n, (const char *)buf);
    }
  }
}

static void setup(void)
{
  print("=== WiFiApi: one command per line, 'b' for the bootloader ===\r\n");
}

static void loop(void)
{
  static uint32_t beat;
  uint8_t         ch;
  char            ip[16];

  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'\r')
    {
      continue;
    }
    if (ch == (uint8_t)'\n')
    {
      Line[LineLen] = '\0';
      handle(Line);
      LineLen = 0U;
    }
    else if (LineLen < (sizeof(Line) - 1U))
    {
      Line[LineLen++] = (char)ch;
    }
  }

  serve_sockets();

  if ((HAL_GetTick() - beat) > BEAT_MS)
  {
    beat = HAL_GetTick();
    say("beat status=%d ip=%s", (int)MayaW476_Status(), ip_str(MayaW476_LocalIP(), ip));
  }
  vTaskDelay(pdMS_TO_TICKS(1U));
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
  * @brief  Keeps the console and the way back to the bootloader alive
  *         whatever the shell is doing, and services the Neuro host.
  */
static void usb_task(void *pArg)
{
  (void)pArg;

  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    (void)BSP_WDG_Refresh();
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    net_service();
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
  }
}

/* Scaffolding */

/**
  * @brief  The BLE UART's interrupt.
  */
void USART6_IRQHandler(void)
{
  BSP_BT_IRQHandler();
}

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
  * @brief  The external memories are not cacheable: a stored setting is
  *         staged in PSRAM for the boot loader.
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
