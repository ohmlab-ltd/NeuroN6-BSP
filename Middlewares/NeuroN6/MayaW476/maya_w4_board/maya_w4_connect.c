/**
 ******************************************************************************
 * @file    maya_w4_connect.c
 * @brief   The Wi-Fi supervisor: one task owns every change to the link.
 *
 * API calls post a request (join, host, disconnect, radio off, scan, power
 * save) and wait for its result. Between requests the task keeps a station
 * joined, rejoining with a growing back-off until MayaW476_Disconnect().
 * Settings are read through maya_w4_creds.h.
 *
 * Threading: wlan_* mutators block on wlan.scan_lock, so they run on the
 * supervisor task, never in wlcmgr_task (the event callback), which only sets
 * flags and reads the assigned address.
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
#include "semphr.h"

#include "maya_w4_creds.h"
#include "maya_w4_txpower.h"   /* the 8 dBm regulatory cap */

#include <string.h>
#include <stdio.h>

/* wifi_nxp public API */
#include "wlan.h"
#include "wifi.h"                 /* wifi_uap_bss_sta_list */
/* SoftAP DHCP server. */
#include "dhcp-server.h"
#include "wm_net.h"               /* net_get_uap_handle, net_dhcp_hostname_set */
#include "lwip/tcpip.h"           /* LOCK_TCPIP_CORE */
#include "lwip/dns.h"
#include "lwip/api.h"             /* netconn_gethostbyname */
#include "lwip/netifapi.h"        /* netifapi_netif_set_default */

/* Defined by the application. */
extern void NeuroN6_DbgPrint(const char *msg);

#define MAYA_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))

/* wifi_nxp profile names, not SSIDs. */
static const char k_sta_profile_name[] = "n6_sta";
static const char k_uap_profile_name[] = "n6_uap";

/* Station rejoin back-off: 2 s after the first failure, doubling to 15 s. */
#define STA_BACKOFF_FIRST_MS   2000u
#define STA_BACKOFF_MAX_MS     15000u
/* A join that makes no progress for this long is wedged and is restarted. */
#define STA_STUCK_MS           60000u
/* SoftAP: a client still in its four-way handshake this long after joining
 * is sent away to join again (keep_softap_clients). */
#define SOFTAP_AUTH_MS         3000u
#define SOFTAP_AUTH_SLOTS      8u

/* Link state */
static volatile int      s_mode                = MAYA_W4_MODE_NONE;
static volatile bool     s_sta_wanted          = false;  /* keep the station joined */
static volatile bool     s_connected           = false;  /* STA: WLAN_REASON_SUCCESS */
static volatile bool     s_softap_started      = false;  /* SoftAP: WLAN_REASON_UAP_SUCCESS */
static volatile bool     s_softap_failed       = false;  /* SoftAP: last start failed */
static volatile bool     s_softap_requested    = false;  /* SoftAP: started until stopped */
static volatile bool     s_dhcp_server_started = false;  /* SoftAP: DHCP server up */
static volatile bool     s_radio_off           = false;  /* MayaW476_End(): deep sleep */
static volatile unsigned s_ps_bits             = 0u;     /* 1 IEEE PS, 2 deep sleep */
static volatile int      s_last_reason         = MAYA_W4_REASON_NONE;
static volatile uint32_t s_initialized_count   = 0U;
static volatile uint32_t s_failed_count        = 0U;
static volatile uint32_t s_client_count        = 0U;
static volatile uint32_t s_link_generation     = 0U;
/* The supervisor aborted a join itself (to scan, or a wedged one): the
 * USER_DISCONNECT that follows is not from MayaW476_Disconnect(). */
static volatile bool     s_internal_disc       = false;
/* SoftAP: whether the access point's security is WPA, which has a handshake,
 * and the clients that have joined and not yet finished it. */
static volatile bool     s_softap_wpa          = false;
typedef struct {
  uint8_t  mac[6];
  uint32_t since_ms;
  bool     used;
} softap_joining_t;
static softap_joining_t  s_joining_clients[SOFTAP_AUTH_SLOTS];

/* Supervisor plumbing */
static TaskHandle_t      s_task       = NULL;
static bool              s_wlan_started = false;   /* wlan_start has succeeded */
static SemaphoreHandle_t s_api_lock   = NULL;   /* one requester at a time */
static SemaphoreHandle_t s_req_done   = NULL;
static volatile int      s_req_op     = 0;
static volatile int      s_req_arg    = 0;
static volatile int      s_req_result = 0;
static volatile uint32_t s_req_seq    = 0U;     /* last request posted */
static volatile uint32_t s_done_seq   = 0U;     /* last request finished */

/* Station keeper */
static uint32_t s_next_attempt_ms = 0U;
static uint32_t s_backoff_ms      = 0U;
static uint32_t s_busy_since_ms   = 0U;

/* Scan */
static SemaphoreHandle_t    s_scan_done = NULL;
static volatile unsigned    s_scan_count = 0U;
static maya_w4_scan_entry_t s_scan[MAYA_W4_SCAN_MAX];
static volatile unsigned    s_scan_n = 0U;

static uint32_t now_ms(void)
{
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Accessors (maya_w4_creds.h), composed by MayaW476_Status() */

/* Usable for sockets: STA associated with a lease, or SoftAP up with its DHCP
 * server running. */
bool maya_w4_wifi_link_ready(void)
{
  return s_connected || (s_softap_started && s_dhcp_server_started);
}

int      maya_w4_wifi_mode               (void) { return s_mode; }
int      maya_w4_wifi_last_reason        (void) { return s_last_reason; }
bool     maya_w4_wifi_sta_connected      (void) { return s_connected; }
uint32_t maya_w4_wifi_failed_count       (void) { return s_failed_count; }
bool     maya_w4_wifi_softap_started     (void) { return s_softap_started; }
bool     maya_w4_wifi_softap_failed      (void) { return s_softap_failed; }
bool     maya_w4_wifi_dhcp_server_started(void) { return s_dhcp_server_started; }
uint32_t maya_w4_wifi_softap_client_count(void) { return s_client_count; }
bool     maya_w4_wifi_radio_off          (void) { return s_radio_off; }
unsigned maya_w4_wifi_ps_bits            (void) { return s_ps_bits; }
uint32_t maya_w4_wifi_link_generation    (void) { return s_link_generation; }

unsigned maya_w4_wifi_scan_results(const maya_w4_scan_entry_t **out)
{
  if (out) *out = s_scan;
  return s_scan_n;
}

/* WifiSecurity (MayaW476.h, 0..6) to wlan_security_type */
static enum wlan_security_type maya_sec_to_wlan(int s)
{
  switch (s) {
    case 0:  return WLAN_SECURITY_NONE;             /* WIFI_SECURITY_NONE */
    case 1:  return WLAN_SECURITY_WEP_OPEN;         /* WIFI_SECURITY_WEP */
    case 2:  return WLAN_SECURITY_WPA;              /* WIFI_SECURITY_WPA */
    case 3:  return WLAN_SECURITY_WPA2;             /* WIFI_SECURITY_WPA2 */
    case 4:  return WLAN_SECURITY_WPA_WPA2_MIXED;   /* WIFI_SECURITY_WPA_WPA2 */
    case 5:  return WLAN_SECURITY_WPA3_SAE;         /* WIFI_SECURITY_WPA3_SAE */
    case 6:  return WLAN_SECURITY_WPA2_WPA3_SAE_MIXED; /* WIFI_SECURITY_WPA2_WPA3 */
    default: return WLAN_SECURITY_WPA2;
  }
}

/* WPA3: the firmware runs SAE; wlan_add_network() sets protected management
 * frames (required for WPA3, offered for WPA2/WPA3). Keep pwe_derivation 0,
 * hunting and pecking (the driver default): with hash-to-element offered,
 * Windows cannot join the access point. */

/* {a,b,c,d} -> uint32_t in network byte order, the form wifi_nxp stores
 * ip.ipv4.* in. */
static uint32_t ip4_to_netorder(const uint8_t ip[4])
{
  return (uint32_t)ip[0]
       | ((uint32_t)ip[1] << 8)
       | ((uint32_t)ip[2] << 16)
       | ((uint32_t)ip[3] << 24);
}

/* {a,b,c,d} -> uint32_t in host order, the form the DHCP server's pool takes. */
static uint32_t ip4_to_host(const uint8_t ip[4])
{
  return ((uint32_t)ip[0] << 24)
       | ((uint32_t)ip[1] << 16)
       | ((uint32_t)ip[2] << 8)
       |  (uint32_t)ip[3];
}

static void print_ip(const char *what, uint32_t ip_be)
{
  char dbg[144];
  snprintf(dbg, sizeof(dbg), "%s%u.%u.%u.%u.\r\n",
           what,
           (unsigned)( ip_be        & 0xFFu), (unsigned)((ip_be >>  8) & 0xFFu),
           (unsigned)((ip_be >> 16) & 0xFFu), (unsigned)((ip_be >> 24) & 0xFFu));
  NeuroN6_DbgPrint(dbg);
}

/* Events: wlcmgr_task context. Only set flags and read the address; a wlan_*
 * mutator here deadlocks on scan_lock. */
static void sta_failed(int why)
{
  /* Any failure of a joined station is a lost link (a departing access
   * point's deauth arrives as an auth failure), and stays one while it
   * rejoins: WL_CONNECTION_LOST holds until the link is back or the
   * application asks for something else. */
  const bool failure = (why == MAYA_W4_REASON_NOT_FOUND || why == MAYA_W4_REASON_AUTH ||
                        why == MAYA_W4_REASON_FAILED    || why == MAYA_W4_REASON_LINK_LOST);
  s_failed_count++;
  if (s_connected) {
    s_connected = false;
    s_link_generation++;
    if (failure) {
      why = MAYA_W4_REASON_LINK_LOST;
      NeuroN6_DbgPrint("[MayaW476] STA link lost - rejoining.\r\n");
    }
  } else if (failure && s_last_reason == MAYA_W4_REASON_LINK_LOST) {
    why = MAYA_W4_REASON_LINK_LOST;
  }
  if (why != MAYA_W4_REASON_NONE) s_last_reason = why;
  maya_w4_set_net_status(false, 0U);
}

static unsigned ps_bit(void *data)
{
  switch ((int)(intptr_t)data) {
    case WLAN_IEEE:            return 1u;
    case WLAN_DEEP_SLEEP:      return 2u;
    case WLAN_IEEE_DEEP_SLEEP: return 3u;
    default:                   return 0u;
  }
}

/* A SoftAP client joined (waiting) or finished its handshake or left (not).
 * Shared by the event callback and the supervisor, so under a critical
 * section. */
static void softap_client_waiting(const uint8_t *mac, bool waiting)
{
  if (mac == NULL) return;
  const uint32_t now = now_ms();
  taskENTER_CRITICAL();
  uint32_t slot = SOFTAP_AUTH_SLOTS;
  for (uint32_t i = 0U; i < SOFTAP_AUTH_SLOTS; ++i) {
    if (s_joining_clients[i].used && memcmp(s_joining_clients[i].mac, mac, 6U) == 0) {
      slot = i;
      break;
    }
    if (!s_joining_clients[i].used && slot == SOFTAP_AUTH_SLOTS) slot = i;
  }
  if (slot < SOFTAP_AUTH_SLOTS) {
    if (waiting) {
      (void)memcpy(s_joining_clients[slot].mac, mac, 6U);
      s_joining_clients[slot].since_ms = now;
    }
    s_joining_clients[slot].used = waiting;
  }
  taskEXIT_CRITICAL();
}

static int wlan_event_callback(enum wlan_event_reason reason, void *data)
{
  switch (reason)
  {
    case WLAN_REASON_INITIALIZED:
      s_initialized_count++;
      break;

    case WLAN_REASON_AUTH_SUCCESS:
      break;  /* intermediate; full success comes via WLAN_REASON_SUCCESS */

    case WLAN_REASON_SUCCESS: {
      struct wlan_ip_config addr;
      (void)memset(&addr, 0, sizeof(addr));
      uint32_t ip_be = 0U;
      if (wlan_get_address(&addr) == WM_SUCCESS) {
        ip_be = addr.ipv4.address;
      }
      s_connected   = true;
      s_last_reason = MAYA_W4_REASON_NONE;
      maya_w4_set_net_status(true, ip_be);
      print_ip("[MayaW476] STA associated + DHCP lease acquired - IP ", ip_be);
      break;
    }

    case WLAN_REASON_NETWORK_NOT_FOUND:
      sta_failed(MAYA_W4_REASON_NOT_FOUND);
      break;

    case WLAN_REASON_NETWORK_AUTH_FAILED:
      sta_failed(MAYA_W4_REASON_AUTH);
      break;

    case WLAN_REASON_ADDRESS_FAILED:
      /* While the access point starts, nothing else is joining: it is the
       * access point's address that failed. */
      if (s_softap_requested && !s_softap_started) {
        s_softap_failed = true;
        NeuroN6_DbgPrint("[MayaW476] SoftAP address FAILED.\r\n");
        break;
      }
      sta_failed(MAYA_W4_REASON_FAILED);
      break;

    case WLAN_REASON_INITIALIZATION_FAILED:
    case WLAN_REASON_CONNECT_FAILED:
      sta_failed(MAYA_W4_REASON_FAILED);
      break;

    case WLAN_REASON_LINK_LOST:
      sta_failed(MAYA_W4_REASON_LINK_LOST);
      break;

    case WLAN_REASON_USER_DISCONNECT:
      if (s_internal_disc) {
        s_internal_disc = false;
        sta_failed(MAYA_W4_REASON_NONE);   /* keep the reason the join last gave */
      } else {
        sta_failed(MAYA_W4_REASON_USER);
      }
      break;

    case WLAN_REASON_UAP_SUCCESS:
      s_softap_started = true;
      s_softap_failed  = false;
      NeuroN6_DbgPrint("[MayaW476] SoftAP up - beaconing.\r\n");
      break;

    case WLAN_REASON_UAP_CLIENT_ASSOC:
      s_client_count++;
      if (s_softap_wpa) softap_client_waiting((const uint8_t *)data, true);
      NeuroN6_DbgPrint("[MayaW476] SoftAP: client joined.\r\n");
      break;

    case WLAN_REASON_UAP_CLIENT_CONN:   /* the handshake finished */
      softap_client_waiting((const uint8_t *)data, false);
      break;

    case WLAN_REASON_UAP_CLIENT_DISSOC:
      if (s_client_count > 0U) s_client_count--;
      if (data != NULL) {
        softap_client_waiting(((const wlan_uap_client_disassoc_t *)data)->sta_addr, false);
      }
      NeuroN6_DbgPrint("[MayaW476] SoftAP: client left.\r\n");
      break;

    case WLAN_REASON_UAP_START_FAILED:
      s_failed_count++;
      s_softap_failed = true;
      NeuroN6_DbgPrint("[MayaW476] SoftAP start FAILED.\r\n");
      break;

    case WLAN_REASON_UAP_STOPPED:
      if (s_softap_started) s_link_generation++;
      s_softap_started   = false;
      s_softap_requested = false;
      s_client_count     = 0U;
      break;

    case WLAN_REASON_PS_ENTER:
      s_ps_bits |= ps_bit(data);
      break;

    case WLAN_REASON_PS_EXIT:
      s_ps_bits &= ~ps_bit(data);
      break;

    default:
      break;
  }

  return 0;
}

/* Profiles */

/* STA: add the target network profile from the application's credentials and, if
 * MayaW476_Config() set one, its fixed address. */
static int add_target_network(const maya_w4_sta_creds_t *c)
{
  struct wlan_network net;
  wlan_initialize_sta_network(&net);

  (void)strncpy(net.name, k_sta_profile_name, sizeof(net.name) - 1U);
  net.name[sizeof(net.name) - 1U] = '\0';

  (void)strncpy(net.ssid, c->ssid ? c->ssid : "", sizeof(net.ssid) - 1U);
  net.ssid[sizeof(net.ssid) - 1U] = '\0';
  net.ssid_specific = 1;

  net.security.type = maya_sec_to_wlan(c->security);

  const char *psk = c->psk ? c->psk : "";
  size_t pw_len = strlen(psk);
  if (pw_len > (sizeof(net.security.password) - 1U)) {
    pw_len = sizeof(net.security.password) - 1U;
  }
  (void)memcpy(net.security.password, psk, pw_len);
  net.security.password_len = pw_len;

  size_t psk_len = strlen(psk);
  if (psk_len > (sizeof(net.security.psk) - 1U)) {
    psk_len = sizeof(net.security.psk) - 1U;
  }
  (void)memcpy(net.security.psk, psk, psk_len);
  net.security.psk_len = (uint8_t)psk_len;

  maya_w4_ip_cfg_t ipc;
  maya_w4_get_sta_ip(&ipc);
  if (ipc.ip != 0U) {
    net.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
    net.ip.ipv4.address   = ipc.ip;
    net.ip.ipv4.gw        = ipc.gateway;
    net.ip.ipv4.netmask   = ipc.netmask;
    net.ip.ipv4.dns1      = ipc.dns1;
    net.ip.ipv4.dns2      = ipc.dns2;
  }

  int rc = wlan_add_network(&net);
  {
    char dbg[128];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] STA add_network rc=%d ssid='%s' sec=%d%s\r\n",
             rc, net.ssid, (int)net.security.type,
             (ipc.ip != 0U) ? " static" : "");
    NeuroN6_DbgPrint(dbg);
  }
  return rc;
}

/* SoftAP: build + add the uAP profile from the application's settings. */
static int add_softap_network(const maya_w4_ap_creds_t *c)
{
  struct wlan_network net;
  wlan_initialize_uap_network(&net);

  /* -1 sentinels: wlan_add_network resolves them to the policy default for
   * the chosen security type. */
  net.security.mfpc = (int8_t)-1;
  net.security.mfpr = (int8_t)-1;

  (void)strncpy(net.name, k_uap_profile_name, sizeof(net.name) - 1U);
  net.name[sizeof(net.name) - 1U] = '\0';

  (void)strncpy(net.ssid, c->ssid ? c->ssid : "", sizeof(net.ssid) - 1U);
  net.ssid[sizeof(net.ssid) - 1U] = '\0';
  net.ssid_specific = 1U;

  /* User-chosen 2.4 GHz channel (0 = auto-select). sec_channel_offset stays 0
   * (HT20), the safe default across regulatory domains. */
  net.channel = (unsigned int)c->channel;
  net.sec_channel_offset = 0;

  net.security.type = maya_sec_to_wlan(c->security);
  if (net.security.type != WLAN_SECURITY_NONE) {
    const char *psk = c->psk ? c->psk : "";
    size_t pw_len = strlen(psk);
    if (pw_len > (sizeof(net.security.password) - 1U)) {
      pw_len = sizeof(net.security.password) - 1U;
    }
    (void)memcpy(net.security.password, psk, pw_len);
    net.security.password_len = pw_len;

    size_t psk_len = strlen(psk);
    if (psk_len > (sizeof(net.security.psk) - 1U)) {
      psk_len = sizeof(net.security.psk) - 1U;
    }
    (void)memcpy(net.security.psk, psk, psk_len);
    net.security.psk_len = (uint8_t)psk_len;
  }

  /* Static IP block, gateway equal to the address (a wlan.c requirement). The
   * DHCP server leases from this subnet once wlan_start_network() is up. */
  uint32_t addr_be = ip4_to_netorder(c->ip);
  net.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
  net.ip.ipv4.address   = addr_be;
  net.ip.ipv4.gw        = addr_be;
  net.ip.ipv4.netmask   = ip4_to_netorder(c->netmask);

#if CONFIG_WIFI_CAPA
  /* No 11AC/11AX: both are compiled out of this 2.4 GHz-only build. */
  net.wlan_capa = WIFI_SUPPORT_LEGACY | WIFI_SUPPORT_11N;
#endif

  return wlan_add_network(&net);
}

/* Regulatory: one SKU for the US, EU and UK, so the radio is pinned to WWSM
 * (World Wide Safe Mode, region code 0x00), legal in all three; also the
 * driver default. Set before add_network so STA and SoftAP inherit it.
 * Non-fatal: a region the module's OTP enforces wins. */
static void maya_w4_lock_region_2g4(void)
{
  int rc = wlan_set_region_code(0x00);   /* 0x00 = "WW " World Wide Safe Mode */
  if (rc == WM_SUCCESS) {
    NeuroN6_DbgPrint("[MayaW476] Region locked to WWSM (2.4 GHz world-safe, "
                     "single SKU US/EU/UK).\r\n");
  } else {
    char dbg[144];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] wlan_set_region_code(WWSM) rc=%d - keeping the "
             "driver/OTP default region (also WWSM unless OTP-enforced).\r\n",
             rc);
    NeuroN6_DbgPrint(dbg);
  }
}

/* Request handlers (supervisor task only) */

static bool wait_flag(volatile bool *flag, bool want, uint32_t ms)
{
  for (uint32_t t = 0U; t < ms; t += 50U) {
    if (*flag == want) return true;
    MAYA_DELAY_MS(50U);
  }
  return *flag == want;
}

/* MayaW476_SetDNS()'s servers override DHCP's. The supervisor reapplies them
 * every tick (a lease renewal replaces them), writing only a changed one.
 * Safe from any task (TCP/IP core lock). */
void maya_w4_wifi_apply_dns(void)
{
  uint32_t d1 = 0U, d2 = 0U;
  maya_w4_get_dns(&d1, &d2);
  if (d1 == 0U && d2 == 0U) return;
  ip_addr_t a;
  LOCK_TCPIP_CORE();
  if (d1 != 0U && ip_addr_get_ip4_u32(dns_getserver(0)) != d1) {
    ip_addr_set_ip4_u32(&a, d1); dns_setserver(0, &a);
  }
  if (d2 != 0U && ip_addr_get_ip4_u32(dns_getserver(1)) != d2) {
    ip_addr_set_ip4_u32(&a, d2); dns_setserver(1, &a);
  }
  UNLOCK_TCPIP_CORE();
}

/* The driver's own scan bookkeeping (wifi.c): a scan it has sent and not yet
 * seen finish, and forgetting one that never will. */
extern int  wifi_event_completion(enum wifi_event event, enum wifi_event_reason result, void *data);
extern bool wifi_user_scan_pending(void);
extern void wifi_user_scan_config_cleanup(void);

/* The driver refuses a scan while another (an abandoned join's included) is
 * under way. One not ended in 6 s never will (the firmware sometimes loses
 * one): clear it as a deauth does, or every later scan and join is refused. */
static void scan_settle(void)
{
  for (uint32_t i = 0U; i < 60U && wifi_user_scan_pending(); ++i) MAYA_DELAY_MS(100U);
  if (!wifi_user_scan_pending()) return;
  NeuroN6_DbgPrint("[MayaW476] a scan never finished - cleared.\r\n");
  wifi_user_scan_config_cleanup();
  (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
  MAYA_DELAY_MS(300U);
}

/* Leave the network and stop rejoining it. */
static void sta_leave(void)
{
  s_sta_wanted = false;
  enum wlan_connection_state st = WLAN_DISCONNECTED;
  if (wlan_get_connection_state(&st) == WM_SUCCESS && st != WLAN_DISCONNECTED) {
    (void)wlan_disconnect();
    for (uint32_t i = 0U; i < 100U; ++i) {
      if (wlan_get_connection_state(&st) == WM_SUCCESS && st == WLAN_DISCONNECTED) break;
      MAYA_DELAY_MS(50U);
    }
  }
  scan_settle();
  if (s_connected) {
    s_connected = false;
    s_link_generation++;
  }
  maya_w4_set_net_status(false, 0U);
}

/* Stop the access point and its DHCP server: one that came up, and one
 * whose start the driver took that never finished, which may yet come up. */
static void stop_ap(void)
{
  if (s_dhcp_server_started) {
    dhcp_server_stop();
    s_dhcp_server_started = false;
  }
  if (s_softap_started || s_softap_requested) {
    (void)wlan_stop_network(k_uap_profile_name);
    (void)wait_flag(&s_softap_started, false, 5000U);
    if (s_softap_started) {
      s_softap_started = false;
      s_link_generation++;
    }
    s_softap_requested = false;
  }
  s_client_count = 0U;
  maya_w4_set_net_status(false, 0U);
}

/* Wake the radio after MayaW476_End(). */
static void radio_on(void)
{
  if (!s_radio_off) return;
  (void)wlan_deepsleepps_off();
  for (uint32_t i = 0U; i < 60U && (s_ps_bits & 2u) != 0u; ++i) MAYA_DELAY_MS(50U);
  s_radio_off = false;
  NeuroN6_DbgPrint("[MayaW476] WiFi radio awake.\r\n");
}

/* The station being kept joined: MayaW476_Begin() again with the same
 * settings (a retry loop) carries on with the join under way. */
static struct {
  char             ssid[65];
  char             psk[WLAN_PASSWORD_MAX_LENGTH + 1U];
  int              security;
  maya_w4_ip_cfg_t ip;
  char             hostname[33];
} s_joining;

static bool same_station(const maya_w4_sta_creds_t *c, const maya_w4_ip_cfg_t *ip)
{
  return strncmp(s_joining.ssid, c->ssid, sizeof(s_joining.ssid)) == 0 &&
         strncmp(s_joining.psk, (c->psk != NULL) ? c->psk : "", sizeof(s_joining.psk)) == 0 &&
         s_joining.security == c->security &&
         memcmp(&s_joining.ip, ip, sizeof(*ip)) == 0 &&
         strncmp(s_joining.hostname, maya_w4_get_hostname(), sizeof(s_joining.hostname)) == 0;
}

static int op_sta(void)
{
  maya_w4_sta_creds_t sta;
  maya_w4_ip_cfg_t ip;
  maya_w4_get_sta_creds(&sta);
  maya_w4_get_sta_ip(&ip);
  if (sta.ssid == NULL || sta.ssid[0] == '\0') return -1;

  if (s_mode == MAYA_W4_MODE_STA && s_sta_wanted && !s_radio_off && same_station(&sta, &ip)) {
    return 0;   /* already joined, or still joining: the station keeper carries on */
  }

  radio_on();
  if ((s_mode == MAYA_W4_MODE_AP) || s_softap_started || s_softap_requested) stop_ap();
  sta_leave();
  (void)netifapi_netif_set_default(net_get_sta_interface());
  (void)wlan_remove_network(k_sta_profile_name);   /* absent the first time */

  /* The name DHCP offers the network; the pointer must outlive the link. */
  (void)net_dhcp_hostname_set((char *)maya_w4_get_hostname());

  int rc = add_target_network(&sta);
  if (rc != WM_SUCCESS) {
    NeuroN6_DbgPrint("[MayaW476] STA add_network FAILED - check SSID/PSK. "
                     "WiFi will not connect.\r\n");
    /* Whatever was hosted or joined has been left above: nothing is now. */
    s_mode        = MAYA_W4_MODE_NONE;
    s_last_reason = MAYA_W4_REASON_FAILED;
    return (rc < 0) ? rc : -rc;
  }

  s_last_reason     = MAYA_W4_REASON_NONE;
  s_mode            = MAYA_W4_MODE_STA;
  s_backoff_ms      = 0U;
  s_busy_since_ms   = 0U;
  s_next_attempt_ms = now_ms();             /* join at once */
  s_sta_wanted      = true;
  (void)snprintf(s_joining.ssid, sizeof(s_joining.ssid), "%s", sta.ssid);
  (void)snprintf(s_joining.psk, sizeof(s_joining.psk), "%s", (sta.psk != NULL) ? sta.psk : "");
  s_joining.security = sta.security;
  s_joining.ip       = ip;
  (void)snprintf(s_joining.hostname, sizeof(s_joining.hostname), "%s", maya_w4_get_hostname());
  NeuroN6_DbgPrint("[MayaW476] STA connecting...\r\n");
  return 0;
}

static int op_ap(void)
{
  maya_w4_ap_creds_t ap;
  maya_w4_get_ap_creds(&ap);

  radio_on();
  if (s_mode == MAYA_W4_MODE_STA) sta_leave();
  stop_ap();
  s_mode = MAYA_W4_MODE_NONE;
  (void)wlan_remove_network(k_uap_profile_name);   /* absent the first time */
  s_softap_failed = false;

  int rc = add_softap_network(&ap);
  if (rc != WM_SUCCESS) {
    char dbg[128];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP add_network FAILED rc=%d. WiFi will not start.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    s_softap_failed = true;
    return -1;
  }

  /* Firmware state, not a profile field: written before every start, as the
   * driver resets it when the access point stops. */
  (void)wlan_uap_set_hidden_ssid(ap.hidden ? 1U : 0U);

  /* Whether a client that joins has a WPA handshake to finish. */
  {
    const enum wlan_security_type sec = maya_sec_to_wlan(ap.security);
    s_softap_wpa = (sec != WLAN_SECURITY_NONE) && (sec != WLAN_SECURITY_WEP_OPEN);
  }
  taskENTER_CRITICAL();
  (void)memset(s_joining_clients, 0, sizeof(s_joining_clients));
  taskEXIT_CRITICAL();

  rc = wlan_start_network(k_uap_profile_name);
  if (rc == WM_SUCCESS) {
    s_softap_requested = true;
  } else {
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "[MayaW476] wlan_start_network FAILED rc=%d.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    s_softap_failed = true;
    return -2;
  }
  NeuroN6_DbgPrint("[MayaW476] wlan_start_network accepted; waiting for UAP_SUCCESS.\r\n");

  /* Wait for UAP_SUCCESS (up to 10 s), or for the start to fail, before
   * standing up the DHCP server. */
  for (uint32_t t = 0U; (t < 10000U) && !s_softap_started && !s_softap_failed; t += 50U) {
    MAYA_DELAY_MS(50U);
  }
  if (!s_softap_started) {
    char dbg[160];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP UAP_SUCCESS never fired. failed=%u clients=%lu\r\n",
             (unsigned)s_softap_failed, (unsigned long)s_client_count);
    NeuroN6_DbgPrint(dbg);
    /* Stopped, so that one that comes up late does not beacon on without a
     * DHCP server, and the next start finds its network free to replace. */
    stop_ap();
    s_softap_failed = true;
    return -3;
  }

  MAYA_DELAY_MS(200U);   /* let the uap netif fully come up */

  s_mode = MAYA_W4_MODE_AP;
  /* The NXP net layer makes the station lwIP's default interface, so
   * broadcast and multicast (mDNS) would leave by the downed station. While
   * hosting the access point is the default; op_sta() hands it back. */
  (void)netifapi_netif_set_default(net_get_uap_interface());
  dhcp_server_set_pool(ip4_to_host(ap.dhcp_first), ip4_to_host(ap.dhcp_last));
  if (dhcp_server_start(net_get_uap_handle()) == WM_SUCCESS) {
    s_dhcp_server_started = true;
    /* The board's own AP IP is the "net ready" address. */
    maya_w4_set_net_status(true, ip4_to_netorder(ap.ip));
    NeuroN6_DbgPrint("[MayaW476] SoftAP DHCP server up - clients can join.\r\n");
  } else {
    /* Clients could join and get no address, and the host server would
     * never be reachable: not an access point anyone can use. */
    NeuroN6_DbgPrint("[MayaW476] SoftAP DHCP server FAILED: the access point "
                     "is stopped.\r\n");
    stop_ap();
    s_mode          = MAYA_W4_MODE_NONE;
    s_softap_failed = true;
    return -4;
  }
  return 0;
}

static int op_disconnect(void)
{
  if (s_mode == MAYA_W4_MODE_STA || s_sta_wanted) sta_leave();
  if (s_mode == MAYA_W4_MODE_AP || s_softap_started || s_softap_requested) stop_ap();
  s_mode        = MAYA_W4_MODE_NONE;
  s_last_reason = MAYA_W4_REASON_USER;
  return 0;
}

/* MayaW476_End(): nothing joined or hosted, and the WiFi half of the module
 * in deep sleep. BLE runs on the same firmware and is not affected. */
static int op_off(void)
{
  (void)op_disconnect();
  if (s_radio_off) return 0;
  int rc = wlan_deepsleepps_on();
  if (rc != WM_SUCCESS) {
    char dbg[96];
    snprintf(dbg, sizeof(dbg), "[MayaW476] deep sleep refused rc=%d.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    return -1;
  }
  for (uint32_t i = 0U; i < 60U && (s_ps_bits & 2u) == 0u; ++i) MAYA_DELAY_MS(50U);
  if ((s_ps_bits & 2u) == 0u) {
    /* The connection manager can accept the request and still refuse it,
     * e.g. with a scan it is waiting on. Report what the module did. */
    NeuroN6_DbgPrint("[MayaW476] the module did not enter deep sleep.\r\n");
    return -2;
  }
  s_radio_off = true;
  NeuroN6_DbgPrint("[MayaW476] WiFi radio off (deep sleep).\r\n");
  return 0;
}

/* Runs in wlcmgr context: only records and signals. */
static int scan_done_cb(unsigned int count)
{
  s_scan_count = count;
  (void)xSemaphoreGive(s_scan_done);
  return 0;
}

static uint8_t scan_security(const struct wlan_scan_result *r)
{
  if (r->wpa3_sae && r->wpa2) return 6u;
  if (r->wpa3_sae)            return 5u;
  if (r->wpa && r->wpa2)      return 4u;
  if (r->wpa2)                return 3u;
  if (r->wpa)                 return 2u;
  if (r->wep)                 return 1u;
  return 0u;
}

/* One scan, waited for. 0 when its results are ready, negative otherwise. */
static int scan_once(void)
{
  (void)xSemaphoreTake(s_scan_done, 0);
  int rc = -1;
  for (uint32_t tries = 0U; tries < 6U; ++tries) {
    rc = wlan_scan(scan_done_cb);
    if (rc == WM_SUCCESS) break;
    MAYA_DELAY_MS(500U);          /* a join in progress holds the scanner */
  }
  if (rc != WM_SUCCESS) return -2;
  if (xSemaphoreTake(s_scan_done, pdMS_TO_TICKS(12000U)) == pdTRUE) return 0;

  /* No answer: the firmware never reported the scan finished, or the
   * connection manager dropped the request without a callback. In the first
   * case its scan lock stays held and every later scan, join and power-save
   * request is refused. Clear it as a deauth does (harmless in the second). */
  NeuroN6_DbgPrint("[MayaW476] scan never finished - abandoned.\r\n");
  wifi_user_scan_config_cleanup();
  (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
  MAYA_DELAY_MS(300U);
  (void)xSemaphoreTake(s_scan_done, 0);
  return -3;
}

static int op_scan(void)
{
  if (s_radio_off) return -1;   /* after MayaW476_End(): MayaW476_Begin() first */

  /* A join in flight holds the scanner (seconds per attempt for an absent
   * network): abort it; the station keeper retries after the scan. */
  if (s_mode == MAYA_W4_MODE_STA && !s_connected) {
    enum wlan_connection_state st = WLAN_DISCONNECTED;
    if (wlan_get_connection_state(&st) == WM_SUCCESS && st != WLAN_DISCONNECTED) {
      s_internal_disc = true;
      (void)wlan_disconnect();
      for (uint32_t i = 0U; i < 60U; ++i) {
        if (wlan_get_connection_state(&st) == WM_SUCCESS && st == WLAN_DISCONNECTED) break;
        MAYA_DELAY_MS(50U);
      }
    }
    s_next_attempt_ms = now_ms() + STA_BACKOFF_FIRST_MS;
  }
  scan_settle();

  int rc = scan_once();
  if (rc == -3) rc = scan_once();   /* abandoned: the next one usually completes */
  if (rc < 0) return rc;

  unsigned n = 0U;
  for (unsigned i = 0U; i < s_scan_count && n < MAYA_W4_SCAN_MAX; ++i) {
    struct wlan_scan_result r;
    if (wlan_get_scan_result(i, &r) != WM_SUCCESS) continue;
    maya_w4_scan_entry_t *e = &s_scan[n];
    size_t len = r.ssid_len < 32U ? r.ssid_len : 32U;
    memcpy(e->ssid, r.ssid, len);
    e->ssid[len] = '\0';
    memcpy(e->bssid, r.bssid, 6);
    e->rssi     = (int8_t)(-(int)r.rssi);   /* the driver reports the magnitude */
    e->channel  = (uint8_t)r.channel;
    e->security = scan_security(&r);
    /* Keep the list strongest first. */
    for (unsigned j = n; j > 0U && s_scan[j].rssi > s_scan[j - 1U].rssi; --j) {
      maya_w4_scan_entry_t t = s_scan[j];
      s_scan[j] = s_scan[j - 1U];
      s_scan[j - 1U] = t;
    }
    n++;
  }
  s_scan_n = n;
  return (int)n;
}

/* IEEE power save: the station sleeps between beacons and the access point
 * buffers its traffic. Only a joined station can do it. */
static int op_ps(int on)
{
  if (s_mode == MAYA_W4_MODE_AP) return -1;
  int rc = on ? wlan_ieeeps_on(WAKE_ON_UNICAST | WAKE_ON_ALL_BROADCAST |
                               WAKE_ON_MULTICAST | WAKE_ON_ARP_BROADCAST)
              : wlan_ieeeps_off();
  return (rc == WM_SUCCESS) ? 0 : -1;
}

static int handle(int op, int arg)
{
  switch (op) {
    case MAYA_W4_OP_STA:        return op_sta();
    case MAYA_W4_OP_AP:         return op_ap();
    case MAYA_W4_OP_DISCONNECT: return op_disconnect();
    case MAYA_W4_OP_OFF:        return op_off();
    case MAYA_W4_OP_SCAN:       return op_scan();
    case MAYA_W4_OP_PS:         return op_ps(arg);
    default:                    return -1;
  }
}

/* Station keeper: rejoin whenever the station is wanted and not joined. */
static void keep_station(void)
{
  if (s_mode != MAYA_W4_MODE_STA || !s_sta_wanted || s_radio_off || s_connected) {
    s_busy_since_ms = 0U;
    if (s_connected) s_backoff_ms = 0U;
    return;
  }

  const uint32_t now = now_ms();
  enum wlan_connection_state st = WLAN_DISCONNECTED;
  if (wlan_get_connection_state(&st) != WM_SUCCESS) return;

  if (st == WLAN_DISCONNECTED) {
    s_busy_since_ms = 0U;
    if ((int32_t)(now - s_next_attempt_ms) >= 0) {
      (void)wlan_connect((char *)k_sta_profile_name);
      s_backoff_ms = (s_backoff_ms == 0U) ? STA_BACKOFF_FIRST_MS
                   : ((s_backoff_ms * 2U > STA_BACKOFF_MAX_MS) ? STA_BACKOFF_MAX_MS
                                                               : s_backoff_ms * 2U);
      s_next_attempt_ms = now + s_backoff_ms;
    }
    return;
  }

  /* Scanning, associating or waiting for an address. A join that makes no
   * progress for a minute is wedged: drop it and try again. */
  if (s_busy_since_ms == 0U) {
    s_busy_since_ms = now;
  } else if ((now - s_busy_since_ms) > STA_STUCK_MS) {
    NeuroN6_DbgPrint("[MayaW476] STA join stuck for 60 s - retrying.\r\n");
    s_internal_disc = true;
    (void)wlan_disconnect();
    s_busy_since_ms   = 0U;
    s_next_attempt_ms = now + STA_BACKOFF_FIRST_MS;
  }
}

/* SoftAP keeper: send away a client that has not finished its four-way
 * handshake SOFTAP_AUTH_MS after joining. The firmware sometimes stalls the
 * handshake, leaving the client joined with no traffic passed; the next
 * association runs it (tens of milliseconds). */
static void keep_softap_clients(void)
{
  if (s_mode != MAYA_W4_MODE_AP || !s_softap_started || !s_softap_wpa) return;

  const uint32_t now = now_ms();
  for (uint32_t i = 0U; i < SOFTAP_AUTH_SLOTS; ++i) {
    uint8_t mac[6];
    bool    due = false;
    taskENTER_CRITICAL();
    if (s_joining_clients[i].used &&
        (int32_t)(now - s_joining_clients[i].since_ms) > (int32_t)SOFTAP_AUTH_MS) {
      (void)memcpy(mac, s_joining_clients[i].mac, sizeof(mac));
      s_joining_clients[i].used = false;
      due = true;
    }
    taskEXIT_CRITICAL();
    if (due) {
      NeuroN6_DbgPrint("[MayaW476] SoftAP: a client joined but never finished the "
                       "four-way handshake - sent it away to join again.\r\n");
      (void)wlan_uap_disconnect_sta(mac);
    }
  }
}

/* Supervisor task, started once wlan_start() has succeeded. */
static void maya_w4_supervisor_task(void *arg)
{
  (void)arg;

  /* Wait for WLAN_REASON_INITIALIZED (poll up to 30 s). */
  for (uint32_t i = 0U; i < 300U; ++i) {
    if (s_initialized_count > 0U) break;
    MAYA_DELAY_MS(100U);
  }
  if (s_initialized_count == 0U) {
    NeuroN6_DbgPrint("[MayaW476] wlan_start did not raise INITIALIZED within 30s.\r\n");
  } else {
    NeuroN6_DbgPrint("[MayaW476] wlan_start initialized; configuring requested mode.\r\n");
  }
  /* Let wlcmgr settle after INITIALIZED. */
  MAYA_DELAY_MS(500U);

  /* Power save off: idle auto-PS would put the chip in deep sleep and starve
   * scan and beacon listening. MayaW476_LowPowerMode() and MayaW476_End() turn
   * it on. */
  (void)wlan_deepsleepps_off();
  (void)wlan_ieeeps_off();
  MAYA_DELAY_MS(500U);

  /* 2.4 GHz regulatory lock (WWSM), before add_network so both modes inherit it. */
  maya_w4_lock_region_2g4();

  /* Regulatory: 8 dBm conducted TX-power cap. Must run after wlan_init applies
   * its regional table and before add_network, so the association and the
   * beacon start capped. */
  maya_w4_apply_tx_power_cap();

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250U));

    const uint32_t seq = s_req_seq;
    if (seq != s_done_seq) {
      const int op = s_req_op, a = s_req_arg;
      s_req_result = handle(op, a);
      s_done_seq = seq;
      (void)xSemaphoreGive(s_req_done);
    }

    keep_station();
    keep_softap_clients();

    if (maya_w4_wifi_link_ready()) {
      maya_w4_wifi_apply_dns();
    }
  }
}

/* Public entry points */

/* Called once wlan_init() succeeded: read the chip MAC, start the connection
 * manager, spawn the supervisor. The first request waits for it to settle. */
int maya_w4_wifi_connect_start(void)
{
  if (s_task != NULL) return 0;

  uint8_t mac[6] = {0};
  if (wlan_get_mac_address(mac) == WM_SUCCESS) {
    char dbg[64];
    (void)snprintf(dbg, sizeof(dbg),
                   "[MayaW476] STA MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    NeuroN6_DbgPrint(dbg);
  }

  if (s_api_lock == NULL)  s_api_lock  = xSemaphoreCreateMutex();
  if (s_req_done == NULL)  s_req_done  = xSemaphoreCreateBinary();
  if (s_scan_done == NULL) s_scan_done = xSemaphoreCreateBinary();
  if (s_api_lock == NULL || s_req_done == NULL || s_scan_done == NULL) {
    NeuroN6_DbgPrint("[MayaW476] WiFi supervisor: out of heap.\r\n");
    return -3;
  }

  /* Once only: the connection manager refuses a second start (a retry after
   * a failed task create skips it). */
  if (!s_wlan_started) {
    int start_rc = wlan_start(wlan_event_callback);
    if (start_rc != WM_SUCCESS) {
      NeuroN6_DbgPrint("[MayaW476] wlan_start FAILED - connection manager "
                       "did not come up.\r\n");
      return -1;
    }
    s_wlan_started = true;
  }

  BaseType_t ok = xTaskCreate(maya_w4_supervisor_task,
                              "maya_w4_conn",
                              1536U,                      /* 6 kB (words) */
                              NULL,
                              tskIDLE_PRIORITY + 2,
                              &s_task);
  if (ok != pdPASS) {
    s_task = NULL;
    NeuroN6_DbgPrint("[MayaW476] supervisor task create FAILED (out of heap?).\r\n");
    return -2;
  }
  return 0;
}

int maya_w4_wifi_request(int op, int arg, uint32_t timeout_ms)
{
  if (s_task == NULL) return -100;
  if (xSemaphoreTake(s_api_lock, portMAX_DELAY) != pdTRUE) return -101;

  (void)xSemaphoreTake(s_req_done, 0);        /* a timed-out request's late give */
  const uint32_t my = s_req_seq + 1U;
  s_req_op  = op;
  s_req_arg = arg;
  s_req_seq = my;
  (void)xTaskNotifyGive(s_task);

  int rc = -102;                              /* timed out: result dropped */
  const uint32_t start = now_ms();
  for (;;) {
    const uint32_t spent = now_ms() - start;
    if (spent >= timeout_ms) break;
    if (xSemaphoreTake(s_req_done, pdMS_TO_TICKS(timeout_ms - spent)) != pdTRUE) break;
    if (s_done_seq == my) { rc = s_req_result; break; }
  }
  (void)xSemaphoreGive(s_api_lock);
  return rc;
}

/* Getters for the public API, from any task but wlcmgr's callback. The BSSID
 * and counters getters each fill one static buffer: not reentrant. */

/* Signal of the joined network in dBm, from the firmware's beacon report. */
int32_t maya_w4_wifi_rssi(void)
{
  if (!s_connected) return 0;
  wlan_rssi_info_t info;
  (void)memset(&info, 0, sizeof(info));
  if (wlan_get_signal_info(&info) != WM_SUCCESS) return 0;
  int32_t r = (info.bcn_rssi_last != 0) ? info.bcn_rssi_last : info.bcn_rssi_avg;
  return (r > 0) ? -r : r;
}

/* The firmware's own version string, as the module reports it. */
const char *maya_w4_wifi_fw_version(void)
{
  const char *v = wlan_get_firmware_version_ext();

  return (v != NULL) ? v : "";
}

/* Transmit power in use (dBm), -1 if unavailable; capped at 8 dBm at
 * start-up. */
int32_t maya_w4_wifi_tx_power(void)
{
  t_u32 power = 0U;

  if (wlan_get_sta_tx_power(&power) != WM_SUCCESS) return -1;
  return (int32_t)power;
}

/* Beacon RSSI and SNR averaged over several packets (steadier than the
 * instantaneous RSSI). Both outputs are optional. */
bool maya_w4_wifi_avg_signal(int32_t *rssi, int32_t *snr)
{
  wlan_rssi_info_t info;

  if (!s_connected) return false;
  /* Not wlan_get_average_signal_strength(): it reports success even when the
   * firmware did not answer. */
  (void)memset(&info, 0, sizeof(info));
  if (wlan_get_signal_info(&info) != WM_SUCCESS) return false;

  if (rssi != NULL) *rssi = (info.bcn_rssi_avg > 0) ? -(int32_t)info.bcn_rssi_avg
                                                    : (int32_t)info.bcn_rssi_avg;
  if (snr  != NULL) *snr  = (int32_t)info.bcn_snr_avg;
  return true;
}

/* 802.11n, one spatial stream, in kbit/s: [bandwidth 20/40][short GI][MCS].
 * The module is 1x1, so a second stream never appears. */
static const uint32_t k_ht_kbit[2][2][8] = {
  { {  6500, 13000, 19500,  26000,  39000,  52000,  58500,  65000 },   /* 20 long  */
    {  7200, 14400, 21700,  28900,  43300,  57800,  65000,  72200 } }, /* 20 short */
  { { 13500, 27000, 40500,  54000,  81000, 108000, 121500, 135000 },   /* 40 long  */
    { 15000, 30000, 45000,  60000,  90000, 120000, 135000, 150000 } }  /* 40 short */
};

/* The legacy set, in the order the firmware indexes it once mlan has closed
 * the gap between the DSSS and OFDM rates. */
static const uint32_t k_legacy_kbit[12] = {
  1000, 2000, 5500, 11000, 6000, 9000, 12000, 18000, 24000, 36000, 48000, 54000
};

static void fill_rate(maya_w4_phy_rate_t *r, uint32_t index, uint32_t bw,
                      uint32_t gi, uint32_t mcs, int format)
{
  if (r == NULL) return;

  r->format    = (uint8_t)format;
  r->short_gi  = (uint8_t)((gi != 0U) ? 1U : 0U);
  r->bandwidth = (uint8_t)((bw == 0U) ? 20U : ((bw == 1U) ? 40U : 80U));
  r->mcs       = (uint8_t)mcs;
  r->kbit_s    = 0U;

  if (format == 0) {
    /* A legacy rate is the index itself. */
    r->mcs = 0U;
    if (index < 12U) r->kbit_s = k_legacy_kbit[index];
  } else if (format == 1) {
    /* Only MCS 0-7 at 20 or 40 MHz is decoded; anything else stays 0. */
    if ((mcs < 8U) && (r->bandwidth <= 40U)) {
      r->kbit_s = k_ht_kbit[(r->bandwidth == 40U) ? 1U : 0U][r->short_gi][mcs];
    }
  }
}

bool maya_w4_wifi_data_rate(maya_w4_phy_rate_t *tx, maya_w4_phy_rate_t *rx)
{
  wifi_ds_rate  ds;
  mlan_bss_type bss;

  if (!s_connected && !s_softap_started) return false;
  bss = (s_mode == MAYA_W4_MODE_AP) ? MLAN_BSS_TYPE_UAP : MLAN_BSS_TYPE_STA;

  memset(&ds, 0, sizeof(ds));
  ds.sub_command = WIFI_DS_GET_DATA_RATE;
  if (wlan_get_data_rate(&ds, bss) != WM_SUCCESS) return false;

  /* Indices from 12 up are MCS 0 up; below that the rate is a legacy one. */
  fill_rate(tx, ds.param.data_rate.tx_data_rate, ds.param.data_rate.tx_bw,
            ds.param.data_rate.tx_gi, ds.param.data_rate.tx_mcs_index,
            (int)ds.param.data_rate.tx_rate_format);
  fill_rate(rx, ds.param.data_rate.rx_data_rate, ds.param.data_rate.rx_bw,
            ds.param.data_rate.rx_gi, ds.param.data_rate.rx_mcs_index,
            (int)ds.param.data_rate.rx_rate_format);
  return true;
}

bool maya_w4_wifi_counters(maya_w4_counters_t *out)
{
  /* 700-odd bytes, too much for this task's stack. */
  static wlan_pkt_stats_t s_stats;
  int                     rc;

  if (out == NULL) return false;
  if (!s_connected && !s_softap_started) return false;

  memset(&s_stats, 0, sizeof(s_stats));
  rc = (s_mode == MAYA_W4_MODE_AP) ? wlan_uap_get_log(&s_stats)
                                   : wlan_get_log(&s_stats);
  if (rc != WM_SUCCESS) return false;

  out->tx_frames      = s_stats.tx_frame;
  out->tx_failed      = s_stats.failed;
  out->tx_retry       = s_stats.retry;
  out->tx_multi_retry = s_stats.multi_retry;
  out->ack_failure    = s_stats.ack_failure;
  out->fcs_error      = s_stats.fcs_error;
  out->rx_multicast   = s_stats.mcast_rx_frame;
  out->beacon_rx      = s_stats.bcn_rcv_cnt;
  out->beacon_miss    = s_stats.bcn_miss_cnt;
  return true;
}

uint32_t maya_w4_wifi_client_list(maya_w4_client_t *out, uint32_t max)
{
  wifi_sta_list_t *sl    = NULL;
  uint32_t         total = 0U;

  if (!s_softap_started) return 0U;
  if (wifi_uap_bss_sta_list(&sl) != WM_SUCCESS || sl == NULL) return 0U;

  if (sl->count > 0) {
    /* A variable-length allocation: the entries follow the count. Both members
     * are byte-wide, so the cast needs no alignment argument. */
    const wifi_sta_info_t *sta = (const wifi_sta_info_t *)(const void *)(sl + 1);
    uint32_t               n;

    total = (uint32_t)sl->count;
    n     = (out != NULL) ? ((total < max) ? total : max) : 0U;

    for (uint32_t i = 0U; i < n; i++) {
      memcpy(out[i].mac, sta[i].mac, 6U);
      out[i].rssi       = (int8_t)sta[i].rssi;
      out[i].power_save = (uint8_t)((sta[i].power_mgmt_status != 0U) ? 1U : 0U);
    }
  }

  /* The caller frees the list (wifi_nxp's own callers leak it). */
#if !CONFIG_MEM_POOLS
  OSA_MemoryFree(sl);
#else
  OSA_MemoryPoolFree(buf_256_MemoryPool, sl);
#endif

  return total;
}

/* One copy of the (large) network record, for the getters below. */
static struct wlan_network s_net_scratch;

bool maya_w4_wifi_bssid(uint8_t out[6])
{
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) {
    return wlan_get_mac_address_uap(out) == WM_SUCCESS;
  }
  if (s_connected && wlan_get_current_network(&s_net_scratch) == WM_SUCCESS) {
    memcpy(out, s_net_scratch.bssid, 6);
    return true;
  }
  return false;
}

bool maya_w4_wifi_mac(uint8_t out[6])
{
  if (s_mode == MAYA_W4_MODE_AP) return wlan_get_mac_address_uap(out) == WM_SUCCESS;
  return wlan_get_mac_address(out) == WM_SUCCESS;
}

uint8_t maya_w4_wifi_channel(void)
{
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) {
    int ch = 0;
    return (wlan_get_uap_channel(&ch) == WM_SUCCESS) ? (uint8_t)ch : 0u;
  }
  if (s_connected && wlan_get_current_network(&s_net_scratch) == WM_SUCCESS) {
    return (uint8_t)s_net_scratch.channel;
  }
  return 0u;
}

/* The interface's address, gateway and netmask, network order. */
bool maya_w4_wifi_addr(uint32_t *ip, uint32_t *gw, uint32_t *mask)
{
  struct wlan_ip_config a;
  (void)memset(&a, 0, sizeof(a));
  int rc = -1;
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) rc = wlan_get_uap_address(&a);
  else if (s_connected)                               rc = wlan_get_address(&a);
  if (rc != WM_SUCCESS) return false;
  if (ip)   *ip   = a.ipv4.address;
  if (gw)   *gw   = a.ipv4.gw;
  if (mask) *mask = a.ipv4.netmask;
  return true;
}

/* Whether an address, in network order, is on the board's own subnet. The
 * responders answer nothing from anywhere else (RFC 6762, 5.5), so the board
 * cannot be used to reflect a larger answer at a spoofed address. */
bool maya_w4_wifi_on_link(uint32_t addr)
{
  uint32_t ip = 0U, mask = 0U;
  if (!maya_w4_wifi_addr(&ip, NULL, &mask) || (mask == 0U)) return false;
  return ((addr ^ ip) & mask) == 0U;
}

uint32_t maya_w4_wifi_dns_server(int n)
{
  if (n < 0 || n >= DNS_MAX_SERVERS) return 0U;
  const ip_addr_t *a = dns_getserver((u8_t)n);
  return (a != NULL && IP_IS_V4(a)) ? ip4_addr_get_u32(ip_2_ip4(a)) : 0U;
}

/* Name lookup through lwIP's resolver (a dotted quad parses directly).
 * Blocks the caller until the answer or the resolver's own timeout. */
int maya_w4_wifi_resolve(const char *host, uint32_t *ip_out)
{
  if (host == NULL || ip_out == NULL) return -1;
  ip_addr_t a;
  if (netconn_gethostbyname(host, &a) != ERR_OK || !IP_IS_V4(&a)) return -1;
  *ip_out = ip4_addr_get_u32(ip_2_ip4(&a));
  return 0;
}
