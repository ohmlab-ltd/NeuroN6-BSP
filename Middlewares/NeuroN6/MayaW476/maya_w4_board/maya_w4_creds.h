/**
 ******************************************************************************
 * @file    maya_w4_creds.h
 * @brief   Library-private bridge between the MayaW476_* API and the Wi-Fi
 *          supervisor task: settings, requests and link state.
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

#ifndef MAYA_W4_CREDS_H
#define MAYA_W4_CREDS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Settings the application made */

/* STA credentials. The strings are library-owned and live for the whole
 * program; never free them. `security` is a WifiSecurity value (0..6). */
typedef struct {
  const char *ssid;
  const char *psk;
  int         security;
} maya_w4_sta_creds_t;

/* SoftAP settings. `ip` is the board's own address as {a,b,c,d}; the connect
 * code converts it to network order. */
typedef struct {
  const char *ssid;
  const char *psk;
  int         security;
  uint8_t     channel;
  bool        hidden;
  uint8_t     ip[4];
  uint8_t     netmask[4];
  uint8_t     dhcp_first[4];   /* lease pool, used when inside the subnet */
  uint8_t     dhcp_last[4];
} maya_w4_ap_creds_t;

/* Station addressing from MayaW476_Config(); ip == 0 means DHCP. Network
 * byte order (wifi_nxp's). */
typedef struct {
  uint32_t ip;
  uint32_t gateway;
  uint32_t netmask;
  uint32_t dns1;
  uint32_t dns2;
} maya_w4_ip_cfg_t;

/* Copy the current settings into the caller's struct. */
void maya_w4_get_sta_creds(maya_w4_sta_creds_t *out);
void maya_w4_get_ap_creds (maya_w4_ap_creds_t  *out);
void maya_w4_get_sta_ip   (maya_w4_ip_cfg_t    *out);

/* DNS servers from MayaW476_SetDNS(), applied once a link is up. 0 = none. */
void maya_w4_get_dns(uint32_t *dns1, uint32_t *dns2);

/* Put them in lwIP's resolver now (MayaW476_SetDNS() with a link up). */
void maya_w4_wifi_apply_dns(void);

/* Name for DHCP and mDNS: MayaW476_SetHostname()'s, else the stored one, else
 * neuron6-xxxxxx from the chip's unique ID. Never NULL; never freed. */
const char *maya_w4_get_hostname(void);

/* Link key (MayaW476_SetLinkKey()'s, else the stored one) copied to `out`;
 * returns its length, 0 when there is none. */
size_t maya_w4_get_link_key(uint8_t *out, size_t cap);

/* TCP port the host server listens on. Default 5000. */
uint16_t maya_w4_get_tcp_port(void);

/* Push link status back to the public accessors. `ipv4_be` is the assigned
 * address in network byte order, or 0 if not yet known. */
void maya_w4_set_net_status(bool net_ready, uint32_t ipv4_be);

/* Requests to the Wi-Fi supervisor task (maya_w4_connect.c) */

enum {
  MAYA_W4_OP_STA = 1,     /* join the network in the STA settings            */
  MAYA_W4_OP_AP,          /* host the access point in the AP settings        */
  MAYA_W4_OP_DISCONNECT,  /* leave the network or stop the access point      */
  MAYA_W4_OP_OFF,         /* disconnect, then put the WiFi radio to sleep    */
  MAYA_W4_OP_SCAN,        /* list the networks in range; returns the count   */
  MAYA_W4_OP_PS           /* arg 1: IEEE power save on, 0: off               */
};

/* Start the connection manager and the supervisor task (once). 0 on success. */
int maya_w4_wifi_connect_start(void);

/* Run one request on the supervisor task and wait up to `timeout_ms` for its
 * result: >= 0 on success, negative on failure or timeout. */
int maya_w4_wifi_request(int op, int arg, uint32_t timeout_ms);

/* Link state */

enum { MAYA_W4_MODE_NONE = 0, MAYA_W4_MODE_STA = 1, MAYA_W4_MODE_AP = 2 };

/* Why the station is not connected, since the last MayaW476_Begin(). */
enum {
  MAYA_W4_REASON_NONE = 0,
  MAYA_W4_REASON_NOT_FOUND,   /* no network with that name answered        */
  MAYA_W4_REASON_AUTH,        /* the network refused the passphrase        */
  MAYA_W4_REASON_FAILED,      /* association or DHCP failed                */
  MAYA_W4_REASON_LINK_LOST,   /* was connected, then lost the link         */
  MAYA_W4_REASON_USER         /* MayaW476_Disconnect() or MayaW476_End()   */
};

int      maya_w4_wifi_mode                  (void);
int      maya_w4_wifi_last_reason           (void);
bool     maya_w4_wifi_sta_connected         (void);   /* associated, with an address */
uint32_t maya_w4_wifi_failed_count          (void);   /* STA failure events, monotonic */
bool     maya_w4_wifi_softap_started        (void);   /* access point beaconing */
bool     maya_w4_wifi_softap_failed         (void);   /* last access point start failed */
bool     maya_w4_wifi_dhcp_server_started   (void);   /* access point DHCP server up */
uint32_t maya_w4_wifi_softap_client_count   (void);   /* stations on the access point */
bool     maya_w4_wifi_radio_off             (void);   /* radio asleep after MayaW476_End() */
unsigned maya_w4_wifi_ps_bits               (void);   /* 1: IEEE power save, 2: deep sleep */
bool     maya_w4_wifi_link_ready            (void);   /* usable for sockets */

/* Bumped each time the link goes down: a task holding sockets closes them
 * and waits for the next link. */
uint32_t maya_w4_wifi_link_generation       (void);

/* Getters for the public API (maya_w4_connect.c) */

int32_t  maya_w4_wifi_rssi(void);                         /* dBm, 0 if not joined */
const char *maya_w4_wifi_fw_version(void);                /* firmware version string  */
int32_t  maya_w4_wifi_tx_power(void);                     /* dBm, -1 if unavailable   */
bool     maya_w4_wifi_avg_signal(int32_t *rssi, int32_t *snr); /* averaged, not instant */

/* One station associated with the access point. */
typedef struct {
  uint8_t mac[6];
  int8_t  rssi;         /* dBm, as the access point hears the client */
  uint8_t power_save;   /* 1 while the client is dozing between beacons */
} maya_w4_client_t;

/* Stations associated with the access point: writes up to `max`, returns the
 * total (may be more); 0 when not hosting. Blocks for the firmware's reply. */
uint32_t maya_w4_wifi_client_list(maya_w4_client_t *out, uint32_t max);

/* One direction of the link, as the firmware last transmitted or received. */
typedef struct {
  uint32_t kbit_s;      /* 0 when the rate format is not one this decodes */
  uint8_t  mcs;         /* 0 for a legacy rate */
  uint8_t  bandwidth;   /* MHz: 20, 40 or 80 */
  uint8_t  short_gi;    /* 1 for the short guard interval */
  uint8_t  format;      /* 0 legacy, 1 HT, 2 VHT, 3 HE */
} maya_w4_phy_rate_t;

bool maya_w4_wifi_data_rate(maya_w4_phy_rate_t *tx, maya_w4_phy_rate_t *rx);

/* Firmware counters for the interface in use, cumulative since radio start. */
typedef struct {
  uint32_t tx_frames;      /* handed to the air */
  uint32_t tx_failed;      /* given up on after the retry limit */
  uint32_t tx_retry;       /* delivered, but not first time */
  uint32_t tx_multi_retry; /* delivered after more than one retry */
  uint32_t ack_failure;    /* sent, no acknowledgement came back */
  uint32_t fcs_error;      /* received with a bad frame check sequence */
  uint32_t rx_multicast;
  uint32_t beacon_rx;
  uint32_t beacon_miss;
} maya_w4_counters_t;

bool maya_w4_wifi_counters(maya_w4_counters_t *out);
bool     maya_w4_wifi_bssid(uint8_t out[6]);              /* joined AP's, or ours */
bool     maya_w4_wifi_mac(uint8_t out[6]);                /* interface in use */
uint8_t  maya_w4_wifi_channel(void);                      /* 0 if none */
bool     maya_w4_wifi_addr(uint32_t *ip, uint32_t *gw, uint32_t *mask);
bool     maya_w4_wifi_on_link(uint32_t addr);              /* network order */
uint32_t maya_w4_wifi_dns_server(int n);                  /* network order, 0 none */
int      maya_w4_wifi_resolve(const char *host, uint32_t *ip_out);   /* 0 ok */

/* Scan results */

#define MAYA_W4_SCAN_MAX  32u

typedef struct {
  char    ssid[33];
  uint8_t bssid[6];
  int8_t  rssi;        /* dBm */
  uint8_t channel;
  uint8_t security;    /* WifiSecurity: 0 open .. 6 WPA2/WPA3 */
} maya_w4_scan_entry_t;

/* The last scan's results, strongest first. Valid until the next scan. */
unsigned maya_w4_wifi_scan_results(const maya_w4_scan_entry_t **out);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_CREDS_H */
