/**
 ******************************************************************************
 * @file    MayaW476.c
 * @brief   Wi-Fi for the Neuro N6 (u-blox MAYA-W476, NXP IW610), in C.
 *
 * Holds the application's settings (read through maya_w4_creds.h), brings the
 * module up once (SDIO, firmware, connection manager, supervisor task, Neuro
 * host server) and turns each call into a request to the supervisor
 * (maya_w4_connect.c).
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "MayaW476.h"

#include <ctype.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "stm32n6xx_hal.h"   /* HAL_GetTick, HAL_GetUIDw*, the RNG */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"

/* Settings and link-status bridge to the Wi-Fi code. */
#include "maya_w4_board/maya_w4_creds.h"
#include "maya_w4_board/maya_w4_sha256.h"   /* maya_w4_wipe */

#include "NeuroN6_FrameQueue.h"   /* TRANSPORT_WIFI_STA / TRANSPORT_WIFI_SOFTAP */
#include "n6_asset.h"             /* the settings a host stored */
#include "n6_host_contract.h"     /* their layout, N6_WIFI_TLV_* */

/* Application settings and link state; the bridge at the end of this file is
 * the only way in. */
static char     s_sta_ssid[64];
/* Any key the driver takes: a WPA2 passphrase (8 to 63), 64 hex digits, or a
 * WPA3 password up to 255 (WLAN_PASSWORD_MAX_LENGTH in wifi_nxp's wlan.h). */
#define MAYA_KEY_CAP                 256U

static char     s_sta_psk[MAYA_KEY_CAP];
static int      s_sta_security = WIFI_SECURITY_WPA2;

static char     s_ap_ssid[64];
static char     s_ap_psk[MAYA_KEY_CAP];
static int      s_ap_security  = WIFI_SECURITY_WPA2;
static uint8_t  s_ap_channel   = 6U;
static bool     s_ap_hidden    = false;
static uint8_t  s_ap_ip[4]     = {192U, 168U, 10U, 1U};
static uint8_t  s_ap_mask[4]   = {255U, 255U, 255U, 0U};

/* MayaW476_Config(): the station's fixed address, 0 for DHCP. */
static uint32_t s_sta_ip, s_sta_gw, s_sta_mask, s_sta_dns1;
/* MayaW476_SetDNS(), 0 for none. */
static uint32_t s_dns1, s_dns2;

static char     s_hostname[33];
static char     s_link_key[65];            /* "" means the stored one */
static uint16_t s_tcp_port     = 5000U;
static uint32_t s_timeout_ms   = 7000U;    /* begin wait */

/* Bring-up stages. */
static bool     s_fw_loaded;               /* SDIO + IW610 firmware */
static bool     s_bringup_failed;          /* the module never answered */
static bool     s_wlcm_started;            /* connection manager + supervisor */

/* Serialises the one-time bring-up and the stored-settings parse, so BLE's
 * start and a begin in another task cannot both power the module. Recursive:
 * the Wi-Fi bring-up includes the firmware's. */
static StaticSemaphore_t s_bringup_storage;
static SemaphoreHandle_t s_bringup_lock;
static bool     s_wlcm_failed;             /* the last start of them failed */
static bool     s_scanned_idle;            /* WL_SCAN_COMPLETED */

static char     s_ipv4_str[16] = "0.0.0.0";
/* Raw IPv4 in wifi_nxp form, read by the IP announce through
 * MayaW476_GetIPv4_BE(). */
static uint32_t s_ipv4_be;

/* Request timeouts. The first request also waits for the connection manager
 * to settle, up to 30 s on a cold start. */
#define START_TIMEOUT_MS  45000U
#define SHORT_TIMEOUT_MS  15000U
#define SCAN_TIMEOUT_MS   30000U

/* Regulatory: 2.4 GHz only (5 GHz is compiled out of the driver). 1..11 is
 * legal under both FCC and ETSI; 0 = auto; above MAX falls back to DEFAULT.
 * Channels 12-13 need an EU/UK-only build with a matching region code. */
#define MAYA_SOFTAP_MAX_CHANNEL      11U
#define MAYA_SOFTAP_DEFAULT_CHANNEL  6U

/* maya_w4_hw.c: GPIO + SDMMC1 + SDIO probe + IW610 firmware download.
 * maya_w4_video_wifi.c: the host server task (TCP clients + discovery). */
extern int maya_w4_sdio_bringup(void);
extern int maya_w4_video_wifi_start(uint16_t tcp_port);

static void copy_bounded(char *dst, size_t cap, const char *src)
{
  size_t n;

  if ((dst == NULL) || (cap == 0U))
  {
    return;
  }
  if (src == NULL)
  {
    dst[0] = '\0';
    return;
  }
  n = strlen(src);
  if (n >= cap)
  {
    n = cap - 1U;
  }
  memcpy(dst, src, n);
  dst[n] = '\0';
}

/* Letters, digits and hyphens, lower case, as DHCP and mDNS want them. */
static void clean_hostname(char *dst, size_t cap, const char *src, size_t n)
{
  size_t o = 0U;

  for (size_t i = 0U; (src != NULL) && (i < n) && (src[i] != '\0') && (o < (cap - 1U)); i++)
  {
    const char c = src[i];

    if ((isalnum((unsigned char)c) != 0) || (c == '-'))
    {
      dst[o++] = (char)tolower((unsigned char)c);
    }
  }
  dst[o] = '\0';
}

static uint8_t checked_channel(uint8_t channel)
{
  char dbg[176];

  if (channel <= MAYA_SOFTAP_MAX_CHANNEL)
  {
    return channel;
  }
  (void)snprintf(dbg, sizeof(dbg),
                 "[MayaW476] SoftAP channel %u is not a valid 2.4 GHz channel "
                 "(1..%u, or 0=auto) - 5 GHz is disabled on this board; using "
                 "channel %u instead.\r\n",
                 (unsigned)channel, (unsigned)MAYA_SOFTAP_MAX_CHANNEL,
                 (unsigned)MAYA_SOFTAP_DEFAULT_CHANNEL);
  NeuroN6_DbgPrint(dbg);
  return MAYA_SOFTAP_DEFAULT_CHANNEL;
}

/* Stored settings */

/* The Wi-Fi record a host stored, read once per boot: storing new settings
 * resets the board. */
typedef struct
{
  bool    read;
  uint8_t mode;             /* wl_stored_mode */
  char    ssid[33];
  char    key[MAYA_KEY_CAP];
  int     security;         /* WifiSecurity, or -1 when absent: WPA2 */
  uint8_t channel;
  char    hostname[33];
  uint8_t link_key[64];
  uint8_t link_key_len;
} stored_wifi_t;

static stored_wifi_t s_stored;

static void tlv_string(char *dst, size_t cap, const uint8_t *v, uint8_t n)
{
  const size_t len = (n < (cap - 1U)) ? n : (cap - 1U);

  memcpy(dst, v, len);
  dst[len] = '\0';
}

/* A no-op before the scheduler starts, when only one context runs. */
static void bringup_lock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    return;
  }
  if (s_bringup_lock == NULL)
  {
    vTaskSuspendAll();
    if (s_bringup_lock == NULL)
    {
      s_bringup_lock = xSemaphoreCreateRecursiveMutexStatic(&s_bringup_storage);
    }
    (void)xTaskResumeAll();
  }
  (void)xSemaphoreTakeRecursive(s_bringup_lock, portMAX_DELAY);
}

static void bringup_unlock(void)
{
  if ((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (s_bringup_lock != NULL))
  {
    (void)xSemaphoreGiveRecursive(s_bringup_lock);
  }
}

static void stored_wifi_parse(void);

/* The settings a host stored, parsed once. */
static const stored_wifi_t *stored_wifi(void)
{
  bringup_lock();
  if (!s_stored.read)
  {
    stored_wifi_parse();
    s_stored.read = true;
  }
  bringup_unlock();
  return &s_stored;
}

static void stored_wifi_parse(void)
{
  static uint8_t rec[512];
  uint32_t       len = 0U;

  s_stored.security = -1;
  s_stored.channel  = MAYA_SOFTAP_DEFAULT_CHANNEL;
  if (!n6_asset_record_read(N6_ASSET_REC_WIFI, rec, sizeof(rec), &len))
  {
    return;
  }

  for (uint32_t i = 0U; ((i + 2U) <= len) && ((i + 2U + rec[i + 1U]) <= len); i += 2U + rec[i + 1U])
  {
    const uint8_t  n = rec[i + 1U];
    const uint8_t *v = &rec[i + 2U];

    switch (rec[i])
    {
      case N6_WIFI_TLV_MODE:
        if ((n == 1U) && ((v[0] == (uint8_t)WL_STORED_STA) || (v[0] == (uint8_t)WL_STORED_AP)))
        {
          s_stored.mode = v[0];
        }
        break;
      case N6_WIFI_TLV_SSID:
        tlv_string(s_stored.ssid, sizeof(s_stored.ssid), v, n);
        break;
      case N6_WIFI_TLV_KEY:
        tlv_string(s_stored.key, sizeof(s_stored.key), v, n);
        break;
      case N6_WIFI_TLV_SECURITY:
        if ((n == 1U) && (v[0] <= (uint8_t)WIFI_SECURITY_WPA2_WPA3))
        {
          s_stored.security = v[0];
        }
        break;
      case N6_WIFI_TLV_CHANNEL:
        if (n == 1U)
        {
          s_stored.channel = v[0];
        }
        break;
      case N6_WIFI_TLV_HOSTNAME:
        clean_hostname(s_stored.hostname, sizeof(s_stored.hostname), (const char *)v, n);
        break;
      case N6_WIFI_TLV_LINK_KEY:
        if ((n >= 8U) && (n <= sizeof(s_stored.link_key)))
        {
          memcpy(s_stored.link_key, v, n);
          s_stored.link_key_len = n;
        }
        break;
      default:
        break;   /* a newer host's field */
    }
  }
  /* Wipe the copy: it holds the passphrase and the link key. */
  maya_w4_wipe(rec, sizeof(rec));
  if (s_stored.ssid[0] == '\0')
  {
    s_stored.mode = (uint8_t)WL_STORED_NONE;   /* no network without a name */
  }
}

/* WPA2 or WPA3 with a passphrase of 8 to 63 characters, or 64 hex digits for
 * WPA2. Anything else is refused with one line. */
static bool security_ok(const char *what, int sec, const char *psk)
{
  const size_t n   = (psk != NULL) ? strlen(psk) : 0U;
  bool         hex = (n == 64U) && (sec == (int)WIFI_SECURITY_WPA2);
  const char  *why = NULL;
  char         msg[176];

  for (size_t i = 0U; hex && (i < n); i++)
  {
    hex = (isxdigit((unsigned char)psk[i]) != 0);
  }
  if ((sec != (int)WIFI_SECURITY_WPA2) && (sec != (int)WIFI_SECURITY_WPA3_SAE) &&
      (sec != (int)WIFI_SECURITY_WPA2_WPA3))
  {
    why = "only WPA2 and WPA3 are supported, so open, WEP and WPA networks are refused";
  }
  else if (n == 0U)
  {
    why = "a passphrase is needed, as open networks are refused";
  }
  else if (((n < 8U) || (n > 63U)) && !hex)
  {
    why = "the passphrase must be 8 to 63 characters";
  }
  if (why == NULL)
  {
    return true;
  }
  (void)snprintf(msg, sizeof(msg), "[MayaW476] %s: %s.\r\n", what, why);
  NeuroN6_DbgPrint(msg);
  return false;
}

/* Take the stored network as the application's own settings; returns the
 * supervisor op that starts it, or -1 when its security is refused. Hosting
 * with no key uses the board's own password. */
static int use_stored(const stored_wifi_t *st)
{
  const bool  ap  = (st->mode == (uint8_t)WL_STORED_AP);
  const int   sec = (st->security >= 0) ? st->security : (int)WIFI_SECURITY_WPA2;
  const char *key = (ap && (st->key[0] == '\0')) ? MayaW476_DevicePassword() : st->key;
  char        msg[160];

  if (!security_ok("the WiFi settings stored on this board", sec, key))
  {
    return -1;
  }
  (void)snprintf(msg, sizeof(msg),
                 "[MayaW476] %s \"%s\", from the WiFi settings stored on this board.\r\n",
                 ap ? "hosting" : "joining", st->ssid);
  NeuroN6_DbgPrint(msg);
  if (ap)
  {
    copy_bounded(s_ap_ssid, sizeof(s_ap_ssid), st->ssid);
    copy_bounded(s_ap_psk,  sizeof(s_ap_psk),  key);
    s_ap_security = sec;
    s_ap_channel  = checked_channel(st->channel);
    return MAYA_W4_OP_AP;
  }
  copy_bounded(s_sta_ssid, sizeof(s_sta_ssid), st->ssid);
  copy_bounded(s_sta_psk,  sizeof(s_sta_psk),  st->key);
  s_sta_security = sec;
  return MAYA_W4_OP_STA;
}

/* Bring-up */

/* Power the module and download its firmware, once. BLE needs no more
 * than this; Wi-Fi also needs ensure_wlan(). */
static int ensure_firmware(void)
{
  int rc = 0;

  bringup_lock();
  if (s_fw_loaded)
  {
    rc = 0;
  }
  else if (s_bringup_failed)
  {
    rc = -1;   /* a missing firmware does not appear later */
  }
  else
  {
    NeuroN6_DbgPrint("[MayaW476] bring-up: powering the module and loading its firmware.\r\n");
    rc = maya_w4_sdio_bringup();
    if (rc != 0)
    {
      s_bringup_failed = true;
    }
    else
    {
      s_fw_loaded = true;
    }
  }
  bringup_unlock();
  return rc;
}

/* Firmware, then the connection manager and the supervisor. The host server
 * starts only from MayaW476_Start(). */
static int ensure_wlan(void)
{
  int rc;

  bringup_lock();
  (void)stored_wifi();   /* read here, in the caller's task, before the Wi-Fi task starts */
  rc = ensure_firmware();
  if ((rc == 0) && !s_wlcm_started)
  {
    rc = maya_w4_wifi_connect_start();
    if (rc != 0)
    {
      NeuroN6_DbgPrint("[MayaW476] wlan_start / supervisor bring-up failed - "
                       "falling back to USB transport.\r\n");
      s_wlcm_failed = true;      /* reported by MayaW476_Status until a start works */
    }
    else
    {
      s_wlcm_failed  = false;
      s_wlcm_started = true;
    }
  }
  bringup_unlock();
  return rc;
}

/* Settings */

void MayaW476_SetTcpPort(uint16_t port)
{
  s_tcp_port = port;
}

void MayaW476_SetTimeout(uint32_t ms)
{
  s_timeout_ms = ms;
}

void MayaW476_Config(uint32_t local_ip, uint32_t dns, uint32_t gateway, uint32_t subnet)
{
  static const uint8_t ap[4]   = {192U, 168U, 10U, 1U};
  static const uint8_t mask[4] = {255U, 255U, 255U, 0U};
  const uint32_t       dot1    = (local_ip & 0x00FFFFFFU) | MAYAW476_IPV4(0U, 0U, 0U, 1U);

  if (local_ip == 0U)
  {
    /* Back to DHCP, and the default access point. */
    s_sta_ip = s_sta_gw = s_sta_mask = s_sta_dns1 = 0U;
    memcpy(s_ap_ip, ap, 4U);
    memcpy(s_ap_mask, mask, 4U);
    return;
  }
  s_sta_ip   = local_ip;
  s_sta_dns1 = (dns != 0U) ? dns : dot1;
  s_sta_gw   = (gateway != 0U) ? gateway : dot1;
  s_sta_mask = (subnet != 0U) ? subnet : MAYAW476_IPV4(255U, 255U, 255U, 0U);
  /* The same address is the access point's, if MayaW476_BeginAP() comes next. */
  memcpy(s_ap_ip, &s_sta_ip, 4U);
  memcpy(s_ap_mask, &s_sta_mask, 4U);
}

void MayaW476_SetDNS(uint32_t dns1, uint32_t dns2)
{
  s_dns1 = dns1;
  s_dns2 = dns2;
  if (maya_w4_wifi_link_ready())
  {
    maya_w4_wifi_apply_dns();
  }
}

void MayaW476_SetHostname(const char *name)
{
  /* Empty: maya_w4_get_hostname() falls back to the stored name or the default. */
  clean_hostname(s_hostname, sizeof(s_hostname), name, (name != NULL) ? strlen(name) : 0U);
}

const char *MayaW476_GetHostname(void)
{
  return maya_w4_get_hostname();
}

bool MayaW476_SetLinkKey(const char *key)
{
  const size_t n = (key != NULL) ? strlen(key) : 0U;

  if ((n != 0U) && ((n < 8U) || (n >= sizeof(s_link_key))))
  {
    return false;
  }
  copy_bounded(s_link_key, sizeof(s_link_key), key);
  return true;
}

/* Joining and hosting */

/* The supervisor's state as a wl_status_t. */
wl_status_t MayaW476_Status(void)
{
  if (s_bringup_failed || s_wlcm_failed)
  {
    return WL_NO_MODULE;
  }
  if (!s_wlcm_started)
  {
    return s_scanned_idle ? WL_SCAN_COMPLETED : WL_IDLE_STATUS;
  }

  switch (maya_w4_wifi_mode())
  {
    case MAYA_W4_MODE_STA:
      if (maya_w4_wifi_sta_connected())
      {
        return WL_CONNECTED;
      }
      switch (maya_w4_wifi_last_reason())
      {
        case MAYA_W4_REASON_NOT_FOUND: return WL_NO_SSID_AVAIL;
        case MAYA_W4_REASON_AUTH:
        case MAYA_W4_REASON_FAILED:    return WL_CONNECT_FAILED;
        case MAYA_W4_REASON_LINK_LOST: return WL_CONNECTION_LOST;
        default:                       return WL_IDLE_STATUS;   /* still joining */
      }
    case MAYA_W4_MODE_AP:
      if (maya_w4_wifi_softap_failed())
      {
        return WL_AP_FAILED;
      }
      if (maya_w4_wifi_softap_started())
      {
        return (maya_w4_wifi_softap_client_count() > 0U) ? WL_AP_CONNECTED : WL_AP_LISTENING;
      }
      return WL_IDLE_STATUS;
    default:
      if (maya_w4_wifi_softap_failed())
      {
        return WL_AP_FAILED;
      }
      if (maya_w4_wifi_last_reason() == MAYA_W4_REASON_USER)
      {
        return WL_DISCONNECTED;
      }
      return s_scanned_idle ? WL_SCAN_COMPLETED : WL_IDLE_STATUS;
  }
}

/* Shared by the begin functions: start the module, ask the supervisor for the
 * mode, then poll the status until it is terminal or the timeout passes. */
static wl_status_t bringup(bool sta)
{
  uint32_t start;

  if (ensure_wlan() != 0)
  {
    return WL_NO_MODULE;
  }
  s_scanned_idle = false;
  if (maya_w4_wifi_request(sta ? MAYA_W4_OP_STA : MAYA_W4_OP_AP, 0, START_TIMEOUT_MS) < 0)
  {
    return sta ? WL_CONNECT_FAILED : WL_AP_FAILED;
  }

  start = HAL_GetTick();
  for (;;)
  {
    const wl_status_t s = MayaW476_Status();

    if (sta && ((s == WL_CONNECTED) || (s == WL_CONNECT_FAILED) ||
                (s == WL_NO_SSID_AVAIL) || (s == WL_NO_MODULE)))
    {
      return s;
    }
    if (!sta && ((s == WL_AP_LISTENING) || (s == WL_AP_CONNECTED) ||
                 (s == WL_AP_FAILED) || (s == WL_NO_MODULE)))
    {
      return s;
    }
    if ((HAL_GetTick() - start) >= s_timeout_ms)
    {
      return s;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));
  }
}

wl_status_t MayaW476_Begin(const char *ssid, const char *psk, WifiSecurity sec)
{
  if (!security_ok("MayaW476_Begin()", (int)sec, psk))
  {
    return WL_CONNECT_FAILED;
  }
  copy_bounded(s_sta_ssid, sizeof(s_sta_ssid), ssid);
  copy_bounded(s_sta_psk,  sizeof(s_sta_psk),  psk);
  s_sta_security = (int)sec;
  return bringup(true);
}

wl_status_t MayaW476_BeginAP(const char *ssid, const char *psk, uint8_t channel,
                             WifiSecurity sec)
{
  if (psk == NULL)
  {
    psk = MayaW476_DevicePassword();
  }
  if (!security_ok("MayaW476_BeginAP()", (int)sec, psk))
  {
    return WL_AP_FAILED;
  }
  copy_bounded(s_ap_ssid, sizeof(s_ap_ssid), ssid);
  copy_bounded(s_ap_psk,  sizeof(s_ap_psk),  psk);
  s_ap_security = (int)sec;
  s_ap_channel  = checked_channel(channel);
  return bringup(false);
}

const char *MayaW476_SoftApPassword(void)
{
  return s_ap_psk;
}

void MayaW476_SetApHidden(bool hidden)
{
  s_ap_hidden = hidden;
}

bool MayaW476_ApHidden(void)
{
  return s_ap_hidden;
}

wl_status_t MayaW476_BeginStored(void)
{
  const stored_wifi_t *st = stored_wifi();
  int                  op;

  if (st->mode == (uint8_t)WL_STORED_NONE)
  {
    NeuroN6_DbgPrint("[MayaW476] no WiFi settings are stored on this board. Store them "
                     "from Neuro Studio, or with harness/bench/n6bench.py wifiset.\r\n");
    return WL_NO_SSID_AVAIL;
  }
  op = use_stored(st);
  if (op < 0)
  {
    return (st->mode == (uint8_t)WL_STORED_AP) ? WL_AP_FAILED : WL_CONNECT_FAILED;
  }
  return bringup(op == MAYA_W4_OP_STA);
}

wl_stored_mode MayaW476_StoredMode(void)
{
  return (wl_stored_mode)stored_wifi()->mode;
}

const char *MayaW476_StoredSSID(void)
{
  return stored_wifi()->ssid;
}

void MayaW476_Disconnect(void)
{
  s_scanned_idle = false;
  if (s_wlcm_started)
  {
    (void)maya_w4_wifi_request(MAYA_W4_OP_DISCONNECT, 0, SHORT_TIMEOUT_MS);
  }
}

void MayaW476_End(void)
{
  s_scanned_idle = false;
  if (s_wlcm_started)
  {
    (void)maya_w4_wifi_request(MAYA_W4_OP_OFF, 0, SHORT_TIMEOUT_MS);
  }
}

bool MayaW476_LowPowerMode(void)
{
  return s_wlcm_started && (maya_w4_wifi_request(MAYA_W4_OP_PS, 1, SHORT_TIMEOUT_MS) == 0);
}

bool MayaW476_NoLowPowerMode(void)
{
  return s_wlcm_started && (maya_w4_wifi_request(MAYA_W4_OP_PS, 0, SHORT_TIMEOUT_MS) == 0);
}

/* The link */

uint32_t MayaW476_LocalIP(void)
{
  return s_ipv4_be;
}

uint32_t MayaW476_SubnetMask(void)
{
  uint32_t mask = 0U;

  return maya_w4_wifi_addr(NULL, NULL, &mask) ? mask : 0U;
}

uint32_t MayaW476_GatewayIP(void)
{
  uint32_t gw = 0U;

  return maya_w4_wifi_addr(NULL, &gw, NULL) ? gw : 0U;
}

uint32_t MayaW476_DnsIP(int n)
{
  return s_wlcm_started ? maya_w4_wifi_dns_server(n) : 0U;
}

const char *MayaW476_Ipv4String(void)
{
  return s_ipv4_str;
}

const char *MayaW476_SSID(void)
{
  switch (maya_w4_wifi_mode())
  {
    case MAYA_W4_MODE_STA: return s_sta_ssid;
    case MAYA_W4_MODE_AP:  return s_ap_ssid;
    default:               return "";
  }
}

int32_t MayaW476_RSSI(void)
{
  return s_wlcm_started ? maya_w4_wifi_rssi() : 0;
}

const char *MayaW476_FirmwareVersion(void)
{
  return s_wlcm_started ? maya_w4_wifi_fw_version() : "";
}

int32_t MayaW476_TxPower(void)
{
  return s_wlcm_started ? maya_w4_wifi_tx_power() : -1;
}

bool MayaW476_AverageSignal(int32_t *rssi, int32_t *snr)
{
  return s_wlcm_started ? maya_w4_wifi_avg_signal(rssi, snr) : false;
}

uint8_t MayaW476_Clients(MayaW476_Client *out, uint8_t max)
{
  uint32_t n;

  if (!s_wlcm_started)
  {
    return 0U;
  }
  n = maya_w4_wifi_client_list((maya_w4_client_t *)out, (uint32_t)max);

  return (n > 255U) ? 255U : (uint8_t)n;
}

bool MayaW476_DataRate(MayaW476_PhyRate *tx, MayaW476_PhyRate *rx)
{
  return s_wlcm_started ? maya_w4_wifi_data_rate((maya_w4_phy_rate_t *)tx,
                                                 (maya_w4_phy_rate_t *)rx)
                        : false;
}

bool MayaW476_GetCounters(MayaW476_Counters *out)
{
  return s_wlcm_started ? maya_w4_wifi_counters((maya_w4_counters_t *)out)
                        : false;
}

uint8_t *MayaW476_BSSID(uint8_t *bssid)
{
  if ((bssid != NULL) && (!s_wlcm_started || !maya_w4_wifi_bssid(bssid)))
  {
    memset(bssid, 0, 6U);
  }
  return bssid;
}

uint8_t MayaW476_Channel(void)
{
  return s_wlcm_started ? maya_w4_wifi_channel() : 0U;
}

uint8_t *MayaW476_MacAddress(uint8_t *mac)
{
  if ((mac != NULL) && (!s_fw_loaded || !maya_w4_wifi_mac(mac)))
  {
    memset(mac, 0, 6U);
  }
  return mac;
}

int MayaW476_HostByName(const char *host, uint32_t *ip)
{
  ip4_addr_t direct;

  if ((host == NULL) || (ip == NULL))
  {
    return 0;
  }
  if (ip4addr_aton(host, &direct) != 0)   /* a dotted quad needs no network */
  {
    *ip = ip4_addr_get_u32(&direct);
    return 1;
  }
  return (s_wlcm_started && (maya_w4_wifi_resolve(host, ip) == 0)) ? 1 : 0;
}

/* Network time (SNTP, RFC 4330) */

#define NTP_TO_UNIX_S    2208988800UL
#define TIME_REFRESH_MS  3600000UL

static char     s_time_server[64] = "pool.ntp.org";
static uint32_t s_time_epoch;   /* seconds at s_time_ms */
static uint32_t s_time_ms;      /* HAL tick at which it was that, 0 = none */

void MayaW476_SetTimeServer(const char *server)
{
  copy_bounded(s_time_server, sizeof(s_time_server), (server != NULL) ? server : "pool.ntp.org");
  s_time_ms = 0U;   /* ask the new server next time */
}

/* Now, counted from the last answer; 0 if there never was one. */
static uint32_t time_now(void)
{
  return (s_time_ms != 0U) ? (s_time_epoch + ((HAL_GetTick() - s_time_ms) / 1000U)) : 0U;
}

/* Transmit timestamp used as a nonce (chip ID, request count, cycle counter
 * and tick, hashed); the server echoes it as the originate timestamp. */
static void ntp_nonce(uint8_t out[8])
{
  static uint32_t  count;
  maya_w4_sha256_t c;
  uint8_t          d[32];
  const uint32_t   mix[5] = {
    HAL_GetUIDw0() ^ HAL_GetUIDw2(), HAL_GetUIDw1(), ++count, DWT->CYCCNT, HAL_GetTick(),
  };

  maya_w4_sha256_init(&c);
  maya_w4_sha256_update(&c, mix, sizeof(mix));
  maya_w4_sha256_final(&c, d);
  memcpy(out, d, 8U);
}

/* One request; takes a synchronised server's answer within 2 s that echoes
 * the nonce (the socket is connected, so only the server's datagrams arrive).
 * SNTP is unauthenticated: an attacker on the path can still forge one. */
static bool ntp_ask(int s)
{
  uint8_t        pkt[48];
  uint8_t        nonce[8];
  const uint32_t sent = HAL_GetTick();

  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0x23U;   /* LI 0, version 4, mode 3 (client) */
  ntp_nonce(nonce);
  memcpy(&pkt[40], nonce, sizeof(nonce));   /* transmit timestamp */
  if (lwip_send(s, pkt, sizeof(pkt), 0) != (int)sizeof(pkt))
  {
    return false;
  }
  while ((HAL_GetTick() - sent) < 2000U)
  {
    const int n = lwip_recv(s, pkt, sizeof(pkt), MSG_DONTWAIT);

    if ((n >= 48) && ((pkt[0] & 0x07U) == 4U) &&           /* a server's answer, */
        ((pkt[0] >> 6) != 3U) &&                             /* its clock synchronised, */
        (pkt[1] >= 1U) && (pkt[1] <= 15U) &&                 /* at a stratum that serves time, */
        ((pkt[40] | pkt[41] | pkt[42] | pkt[43]) != 0U) &&   /* with a time in it, */
        (memcmp(&pkt[24], nonce, sizeof(nonce)) == 0))       /* to this request */
    {
      const uint32_t secs = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) |
                            ((uint32_t)pkt[42] << 8)  |  (uint32_t)pkt[43];
      const uint32_t frac = ((uint32_t)pkt[44] << 24) | ((uint32_t)pkt[45] << 16) |
                            ((uint32_t)pkt[46] << 8)  |  (uint32_t)pkt[47];
      const uint32_t frac_ms = (uint32_t)(((uint64_t)frac * 1000U) >> 32);

      /* The server stamped mid round trip, frac_ms into its second. */
      s_time_epoch = secs - NTP_TO_UNIX_S;
      s_time_ms    = (sent + ((HAL_GetTick() - sent) / 2U)) - frac_ms;
      if (s_time_ms == 0U)
      {
        s_time_ms = 1U;   /* 0 means no answer yet */
      }
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));
  }
  return false;
}

uint32_t MayaW476_GetTime(void)
{
  struct sockaddr_in to;
  uint32_t           server = 0U;
  int                s;

  if ((s_time_ms != 0U) && ((HAL_GetTick() - s_time_ms) < TIME_REFRESH_MS))
  {
    return time_now();
  }
  if (!maya_w4_wifi_link_ready() || (MayaW476_HostByName(s_time_server, &server) == 0))
  {
    return time_now();
  }
  s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0)
  {
    return time_now();
  }
  memset(&to, 0, sizeof(to));
  to.sin_family      = AF_INET;
  to.sin_port        = lwip_htons(123U);
  to.sin_addr.s_addr = server;
  if (lwip_connect(s, (const struct sockaddr *)&to, sizeof(to)) == 0)
  {
    for (int attempt = 0; (attempt < 3) && !ntp_ask(s); attempt++)
    {
    }
  }
  (void)lwip_close(s);
  return time_now();
}

/* Scanning */

int8_t MayaW476_ScanNetworks(void)
{
  int n;

  if (ensure_wlan() != 0)
  {
    return -1;
  }
  n = maya_w4_wifi_request(MAYA_W4_OP_SCAN, 0, SCAN_TIMEOUT_MS);
  if (n < 0)
  {
    return -1;
  }
  if (maya_w4_wifi_mode() == MAYA_W4_MODE_NONE)
  {
    s_scanned_idle = true;
  }
  return (int8_t)n;
}

static const maya_w4_scan_entry_t *scan_entry(uint8_t i)
{
  const maya_w4_scan_entry_t *all = NULL;
  const unsigned              n   = maya_w4_wifi_scan_results(&all);

  return ((all != NULL) && (i < n)) ? &all[i] : NULL;
}

const char *MayaW476_ScanSSID(uint8_t i)
{
  const maya_w4_scan_entry_t *e = scan_entry(i);

  return (e != NULL) ? e->ssid : "";
}

int32_t MayaW476_ScanRSSI(uint8_t i)
{
  const maya_w4_scan_entry_t *e = scan_entry(i);

  return (e != NULL) ? e->rssi : 0;
}

uint8_t MayaW476_ScanEncryption(uint8_t i)
{
  const maya_w4_scan_entry_t *e = scan_entry(i);

  if (e == NULL)
  {
    return ENC_TYPE_UNKNOWN;
  }
  switch (e->security)
  {
    case WIFI_SECURITY_NONE:     return ENC_TYPE_NONE;
    case WIFI_SECURITY_WEP:      return ENC_TYPE_WEP;
    case WIFI_SECURITY_WPA:      return ENC_TYPE_TKIP;
    case WIFI_SECURITY_WPA2:     return ENC_TYPE_CCMP;
    case WIFI_SECURITY_WPA3_SAE: return ENC_TYPE_WPA3;
    case WIFI_SECURITY_WPA_WPA2:
    case WIFI_SECURITY_WPA2_WPA3: return ENC_TYPE_AUTO;
    default:                     return ENC_TYPE_UNKNOWN;
  }
}

uint8_t *MayaW476_ScanBSSID(uint8_t i, uint8_t *bssid)
{
  const maya_w4_scan_entry_t *e = scan_entry(i);

  if (bssid != NULL)
  {
    if (e != NULL)
    {
      memcpy(bssid, e->bssid, 6U);
    }
    else
    {
      memset(bssid, 0, 6U);
    }
  }
  return bssid;
}

uint8_t MayaW476_ScanChannel(uint8_t i)
{
  const maya_w4_scan_entry_t *e = scan_entry(i);

  return (e != NULL) ? e->channel : 0U;
}

/* Bridge to the Wi-Fi code (maya_w4_creds.h) */

void maya_w4_get_sta_creds(maya_w4_sta_creds_t *out)
{
  if (out != NULL)
  {
    out->ssid     = s_sta_ssid;   /* always NUL-terminated (copy_bounded) */
    out->psk      = s_sta_psk;
    out->security = s_sta_security;
  }
}

void maya_w4_get_ap_creds(maya_w4_ap_creds_t *out)
{
  if (out != NULL)
  {
    out->ssid     = s_ap_ssid;
    out->psk      = s_ap_psk;
    out->security = s_ap_security;
    out->channel  = s_ap_channel;
    out->hidden   = s_ap_hidden;
    memcpy(out->ip,      s_ap_ip,   4U);
    memcpy(out->netmask, s_ap_mask, 4U);
    /* .10 to .99 beside the access point's own address; when those are not
     * in its subnet, the server leases from the whole subnet. */
    memcpy(out->dhcp_first, s_ap_ip, 3U);
    memcpy(out->dhcp_last,  s_ap_ip, 3U);
    out->dhcp_first[3] = 10U;
    out->dhcp_last[3]  = 99U;
  }
}

void maya_w4_get_sta_ip(maya_w4_ip_cfg_t *out)
{
  if (out != NULL)
  {
    out->ip      = s_sta_ip;
    out->gateway = s_sta_gw;
    out->netmask = s_sta_mask;
    out->dns1    = s_sta_dns1;
    out->dns2    = 0U;
  }
}

void maya_w4_get_dns(uint32_t *dns1, uint32_t *dns2)
{
  if (dns1 != NULL)
  {
    *dns1 = s_dns1;
  }
  if (dns2 != NULL)
  {
    *dns2 = s_dns2;
  }
}

const char *maya_w4_get_hostname(void)
{
  static char s_default[16];

  if (s_hostname[0] != '\0')
  {
    return s_hostname;
  }
  if (stored_wifi()->hostname[0] != '\0')
  {
    return s_stored.hostname;
  }
  if (s_default[0] == '\0')
  {
    /* Stable per board: the chip's unique ID, not the MAC, so the name is
     * known before the radio is up. */
    const uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

    (void)snprintf(s_default, sizeof(s_default), "neuron6-%06lx", (unsigned long)(uid & 0xFFFFFFU));
  }
  return s_default;
}

size_t maya_w4_get_link_key(uint8_t *out, size_t cap)
{
  size_t         n = strlen(s_link_key);
  const uint8_t *k = (const uint8_t *)s_link_key;

  if (n == 0U)
  {
    n = stored_wifi()->link_key_len;
    k = s_stored.link_key;
  }
  if (n > cap)
  {
    return 0U;
  }
  memcpy(out, k, n);
  return n;
}

uint16_t maya_w4_get_tcp_port(void)
{
  return s_tcp_port;
}

void maya_w4_set_net_status(bool net_ready, uint32_t ipv4_be)
{
  s_ipv4_be = net_ready ? ipv4_be : 0U;   /* feeds MayaW476_GetIPv4_BE() */
  if (ipv4_be != 0U)
  {
    /* wifi_nxp form: on little-endian ARM the bytes read [a][b][c][d]. */
    (void)snprintf(s_ipv4_str, sizeof(s_ipv4_str), "%u.%u.%u.%u",
                   (unsigned)(ipv4_be & 0xFFU), (unsigned)((ipv4_be >> 8) & 0xFFU),
                   (unsigned)((ipv4_be >> 16) & 0xFFU), (unsigned)((ipv4_be >> 24) & 0xFFU));
  }
  else if (!net_ready)
  {
    copy_bounded(s_ipv4_str, sizeof(s_ipv4_str), "0.0.0.0");
  }
}

/* The IP announce's accessors. The address is zero until the board's own
 * access point is up, as the host server runs on it only. */
uint32_t MayaW476_GetIPv4_BE(void)
{
  return (maya_w4_wifi_mode() == MAYA_W4_MODE_AP) ? s_ipv4_be : 0U;
}
uint16_t MayaW476_GetIPv4_Port(void) { return s_tcp_port; }

/* The USB-only META_BOARD_SOFTAP value: { u8 name length, name, u8
 * passphrase length, passphrase } while the access point is up. */
uint32_t MayaW476_SoftApAnnounce(uint8_t *out, uint32_t cap)
{
  const size_t sn = strlen(s_ap_ssid);
  const size_t pn = strlen(s_ap_psk);

  if ((maya_w4_wifi_mode() != MAYA_W4_MODE_AP) || !maya_w4_wifi_softap_started() ||
      (sn == 0U) || (sn > 32U) || (pn == 0U) || ((2U + sn + pn) > cap))
  {
    return 0U;
  }
  out[0] = (uint8_t)sn;
  memcpy(&out[1], s_ap_ssid, sn);
  out[1U + sn] = (uint8_t)pn;
  memcpy(&out[2U + sn], s_ap_psk, pn);
  return (uint32_t)(2U + sn + pn);
}

/* The board's own password */

/* Fill out from the hardware RNG. A stopped RNG is started (with clock error
 * detection, as ST's HAL does) and stopped again; one already running for TLS
 * is only read. False on a seed or clock error. */
static bool hw_random(uint8_t *out, size_t n)
{
  const bool clocked = (__HAL_RCC_RNG_IS_CLK_ENABLED() != 0U);
  const bool running = clocked && ((RNG->CR & RNG_CR_RNGEN) != 0U);
  bool       ok      = true;
  uint32_t   t0;

  if (!running)
  {
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR = (RNG->CR & ~(RNG_CR_RNGEN | RNG_CR_CED)) | RNG_CR_CONDRST;   /* clock check on */
    RNG->CR &= ~RNG_CR_CONDRST;
    t0 = HAL_GetTick();
    while (ok && ((RNG->CR & RNG_CR_CONDRST) != 0U))
    {
      ok = ((HAL_GetTick() - t0) <= 10U);
    }
    RNG->SR = 0U;   /* clears the seed and clock error flags */
    RNG->CR |= RNG_CR_RNGEN;
  }
  for (size_t i = 0U; ok && (i < n); )
  {
    uint32_t w;

    t0 = HAL_GetTick();
    while (ok && ((RNG->SR & RNG_SR_DRDY) == 0U))
    {
      ok = ((RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)) == 0U) && ((HAL_GetTick() - t0) <= 10U);
    }
    if (!ok)
    {
      break;
    }
    w = RNG->DR;
    /* A word read during a seed or clock error is not used. */
    ok = ((RNG->SR & (RNG_SR_SEIS | RNG_SR_CEIS)) == 0U);
    for (uint32_t b = 0U; ok && (b < 4U) && (i < n); b++)
    {
      out[i++] = (uint8_t)(w >> (8U * b));
    }
  }
  if (!running)
  {
    RNG->CR &= ~RNG_CR_RNGEN;
    if (!clocked)
    {
      __HAL_RCC_RNG_CLK_DISABLE();
    }
  }
  return ok;
}

/* xxxx-xxxx-xxxx-xxxx from 32 letters and digits, 80 random bits; l, o, 0 and
 * 1 look alike and are left out. */
static bool new_password(char out[20])
{
  static const char k_chars[] = "abcdefghijkmnpqrstuvwxyz23456789";
  uint8_t           r[16];
  size_t            o = 0U;

  if (!hw_random(r, sizeof(r)) && !hw_random(r, sizeof(r)))
  {
    return false;
  }
  for (size_t i = 0U; i < sizeof(r); i++)
  {
    if ((i > 0U) && ((i % 4U) == 0U))
    {
      out[o++] = '-';
    }
    out[o++] = k_chars[r[i] & 31U];
  }
  out[o] = '\0';
  maya_w4_wipe(r, sizeof(r));
  return true;
}

static bool printable(const char *s, uint32_t n)
{
  for (uint32_t i = 0U; i < n; i++)
  {
    if (((unsigned char)s[i] < 0x20U) || ((unsigned char)s[i] > 0x7EU))
    {
      return false;
    }
  }
  return true;
}

const char *MayaW476_DevicePassword(void)
{
  static char s_pass[64];
  uint32_t    n = 0U;
  uint32_t    last;

  if (s_pass[0] != '\0')
  {
    return s_pass;
  }
  if (n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, s_pass, sizeof(s_pass) - 1U, &n) &&
      (n >= 8U) && printable(s_pass, n))
  {
    s_pass[n] = '\0';
    return s_pass;
  }
  if (!new_password(s_pass))
  {
    s_pass[0] = '\0';
    NeuroN6_DbgPrint("[MayaW476] the hardware random number generator failed, so the board "
                     "has no SoftAP password.\r\n");
    return s_pass;
  }

  /* A stage the boot loader left, or its error, means a restart would not
   * store it either. */
  last = n6_asset_last_result();
  if (n6_asset_pending() || ((last != 0U) && (last != N6_ASSET_RESULT_OK)) ||
      !n6_asset_record_stage(N6_ASSET_REC_AP_PASSWORD, s_pass, (uint32_t)strlen(s_pass)))
  {
    NeuroN6_DbgPrint("[MayaW476] the board's SoftAP password could not be stored, so it "
                     "changes at the next restart.\r\n");
    return s_pass;
  }
  NeuroN6_DbgPrint("[MayaW476] storing the board's new SoftAP password: the board restarts "
                   "once.\r\n");
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    vTaskDelay(pdMS_TO_TICKS(200U));
  }
  n6_asset_apply();
  return s_pass;
}

/* Hooks */

int MayaW476_RadioStart(void)
{
  return ensure_firmware();
}

int MayaW476_Start(int transport_mode)
{
  const stored_wifi_t *st;
  int                  mode;
  int                  rc;
  char                 msg[160];

  if (transport_mode != TRANSPORT_WIFI_SOFTAP)
  {
    NeuroN6_DbgPrint("[MayaW476] the WiFi stream runs on the board's own access point only: "
                     "use TRANSPORT_WIFI_SOFTAP.\r\n");
    return -3;
  }
  if (ensure_wlan() != 0)
  {
    return -1;
  }

  /* An access point the application started with MayaW476_BeginAP() is
   * kept. */
  mode = maya_w4_wifi_mode();
  if (mode == MAYA_W4_MODE_STA)
  {
    NeuroN6_DbgPrint("[MayaW476] the board joined a network with MayaW476_Begin(), and the WiFi "
                     "stream runs on its own access point only.\r\n");
    return -3;
  }
  if (mode != MAYA_W4_MODE_AP)
  {
    /* The application's settings, else the stored ones. */
    st = stored_wifi();
    if ((s_ap_ssid[0] == '\0') && (st->mode == (uint8_t)WL_STORED_AP) && (use_stored(st) < 0))
    {
      return -2;
    }
    if (s_ap_ssid[0] == '\0')
    {
      NeuroN6_DbgPrint("[MayaW476] the WiFi stream needs an access point, and none is set: call "
                       "MayaW476_BeginAP(), or store WiFi settings on the board from Neuro "
                       "Studio.\r\n");
      return -2;
    }
    if (s_ap_psk[0] == '\0')
    {
      copy_bounded(s_ap_psk, sizeof(s_ap_psk), MayaW476_DevicePassword());
    }
    if (!security_ok("MayaW476_Start()", s_ap_security, s_ap_psk))
    {
      return -2;
    }
    (void)snprintf(msg, sizeof(msg),
                   "[MayaW476] starting the access point '%s' on channel %u, tcp_port=%u.\r\n",
                   s_ap_ssid, (unsigned)s_ap_channel, (unsigned)s_tcp_port);
    NeuroN6_DbgPrint(msg);
    rc = maya_w4_wifi_request(MAYA_W4_OP_AP, 0, START_TIMEOUT_MS);
    if (rc < 0)
    {
      NeuroN6_DbgPrint("[MayaW476] could not start the access point - "
                       "falling back to USB transport.\r\n");
      return rc;
    }
  }
  if (maya_w4_video_wifi_start(s_tcp_port) != 0)
  {
    return -1;
  }
  NeuroN6_DbgPrint("[MayaW476] WiFi transport active. Point Neuro "
                   "Studio at the board's IP on the configured TCP port.\r\n");
  return 0;
}
