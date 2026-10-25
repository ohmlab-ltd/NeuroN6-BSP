/**
 ******************************************************************************
 * @file    MayaW476.h
 * @brief   Wi-Fi for the Neuro N6 (u-blox MAYA-W476, NXP IW610), in C.
 *
 * MayaW476_Start() runs the Neuro host server on the board's own access point:
 * Neuro Studio connects over TCP for the stream and live controls, and finds
 * the board by UDP broadcast or mDNS. For the application's own connections
 * use lwIP sockets (lwip/sockets.h) once MayaW476_Status() reports a link.
 *
 * Addresses are uint32_t with the first octet in the low byte (wifi_nxp's
 * form); MAYAW476_IPV4(192, 168, 1, 20) writes one. The application supplies
 * NeuroN6_DbgPrint(), and NeuroN6_NetAttach() and NeuroN6_IpAnnounceFrame()
 * (NeuroN6_FrameQueue.h) for the host server.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_W476_H
#define MAYA_W476_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IPv4 address in this API's form (first octet in the low byte). */
#define MAYAW476_IPV4(a, b, c, d)                                   \
  ((uint32_t)(uint8_t)(a)         | ((uint32_t)(uint8_t)(b) << 8) | \
   ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24))

/* Security (wifi_nxp's WLAN_SECURITY_* values). The begin functions take WPA2
 * and WPA3 only; the others describe scan results. */
typedef enum {
  WIFI_SECURITY_NONE        = 0,   /* open */
  WIFI_SECURITY_WEP         = 1,   /* not recommended */
  WIFI_SECURITY_WPA         = 2,   /* legacy */
  WIFI_SECURITY_WPA2        = 3,   /* most home networks */
  WIFI_SECURITY_WPA_WPA2    = 4,   /* mixed mode */
  WIFI_SECURITY_WPA3_SAE    = 5,   /* WPA3 personal */
  WIFI_SECURITY_WPA2_WPA3   = 6,   /* mixed WPA2/WPA3 */
} WifiSecurity;

/* Link status. */
typedef enum {
  WL_IDLE_STATUS      = 0,   /* before a begin, or a station still joining */
  WL_NO_SSID_AVAIL    = 1,   /* STA: no network with that name answered */
  WL_SCAN_COMPLETED   = 2,   /* idle, and MayaW476_ScanNetworks() has results */
  WL_CONNECTED        = 3,   /* STA associated + DHCP lease acquired */
  WL_CONNECT_FAILED   = 4,   /* STA connect attempt failed (bad creds, etc.) */
  WL_CONNECTION_LOST  = 5,   /* STA was connected and lost the link; rejoining */
  WL_DISCONNECTED     = 6,   /* MayaW476_Disconnect() or MayaW476_End() */
  WL_AP_LISTENING     = 7,   /* SoftAP up, no clients yet */
  WL_AP_CONNECTED     = 8,   /* SoftAP up + at least one client associated */
  WL_AP_FAILED        = 9,   /* SoftAP failed to start */
  WL_NO_MODULE        = 255  /* IW610 / SDIO bring-up or wlan_init failed */
} wl_status_t;

/* A scanned network's encryption (the WiFiNINA / WiFi101 values). */
typedef enum {
  ENC_TYPE_TKIP    = 2,     /* WPA  */
  ENC_TYPE_WPA     = 2,
  ENC_TYPE_CCMP    = 4,     /* WPA2 */
  ENC_TYPE_WPA2    = 4,
  ENC_TYPE_WEP     = 5,
  ENC_TYPE_WPA3    = 6,
  ENC_TYPE_NONE    = 7,     /* open */
  ENC_TYPE_AUTO    = 8,     /* mixed WPA/WPA2 or WPA2/WPA3 */
  ENC_TYPE_UNKNOWN = 255
} wl_enc_type;

/* The Wi-Fi settings a host stored on the board. */
typedef enum {
  WL_STORED_NONE = 0,   /* nothing stored */
  WL_STORED_STA  = 1,   /* join the stored network */
  WL_STORED_AP   = 2    /* host the stored network */
} wl_stored_mode;

/* Joining and hosting */

/* Join a WPA2 or WPA3 network. psk: 8 to 63 characters, or 64 hex digits for
 * WPA2; anything else, open included, returns WL_CONNECT_FAILED. Blocks up to
 * MayaW476_SetTimeout() (default 7000 ms), then keeps trying and rejoins in
 * the background until MayaW476_Disconnect() or MayaW476_End(). Called again
 * with other settings, or after MayaW476_BeginAP(), it leaves the old network
 * first. The strings are copied. */
wl_status_t MayaW476_Begin(const char *ssid, const char *psk, WifiSecurity sec);

/* Host a WPA2 or WPA3 network at 192.168.10.1/24 (or MayaW476_Config()'s
 * address), leasing .10 to .99 (the whole subnet when its mask leaves those
 * out). psk as for MayaW476_Begin(); NULL uses MayaW476_DevicePassword(); a
 * refused key returns WL_AP_FAILED. Blocks up to the timeout.
 * 2.4 GHz only: channel 1..11, 0 = automatic, above 11 falls back to 6. */
wl_status_t MayaW476_BeginAP(const char *ssid, const char *psk, uint8_t channel,
                             WifiSecurity sec);

/* Leave the SSID out of the beacon (not a security measure: a joining client
 * still sends it). Applies from the next MayaW476_BeginAP(); default off. */
void MayaW476_SetApHidden(bool hidden);
bool MayaW476_ApHidden(void);

/* The board's own SoftAP password, xxxx-xxxx-xxxx-xxxx (16 random characters
 * from the hardware generator), kept in asset store record 3. The first call
 * stores it and restarts the board once; if the boot loader cannot store it,
 * it lasts until the next restart. "" if the generator fails. */
const char *MayaW476_DevicePassword(void);

/* Passphrase of the access point hosted or last set up; "" before any. */
const char *MayaW476_SoftApPassword(void);

/* Stations associated with the access point. Fills up to max entries and
 * returns the total, which may exceed max; 0 when not hosting. Blocks on the
 * firmware: a diagnostic, not for polling. */
typedef struct {
  uint8_t mac[6];
  int8_t  rssi;         /* dBm, as the access point hears the client */
  uint8_t power_save;   /* 1 while the client is dozing between beacons */
} MayaW476_Client;

uint8_t MayaW476_Clients(MayaW476_Client *out, uint8_t max);

/* Rates of the last frame sent and received (at or below the negotiated
 * rate). kbit_s is decoded for legacy and 802.11n rates only. Either pointer
 * may be NULL. False when nothing is joined or hosted. */
typedef struct {
  uint32_t kbit_s;      /* 0 when the rate format is not one this decodes */
  uint8_t  mcs;         /* 0 for a legacy rate */
  uint8_t  bandwidth;   /* MHz: 20, 40 or 80 */
  uint8_t  short_gi;    /* 1 for the short guard interval */
  uint8_t  format;      /* 0 legacy, 1 HT, 2 VHT, 3 HE */
} MayaW476_PhyRate;

bool MayaW476_DataRate(MayaW476_PhyRate *tx, MayaW476_PhyRate *rx);

/* Firmware counters for the interface in use, cumulative since the radio
 * started. Retries and ACK failures rise well before a link drops. */
typedef struct {
  uint32_t tx_frames;
  uint32_t tx_failed;
  uint32_t tx_retry;
  uint32_t tx_multi_retry;
  uint32_t ack_failure;
  uint32_t fcs_error;
  uint32_t rx_multicast;
  uint32_t beacon_rx;
  uint32_t beacon_miss;
} MayaW476_Counters;

bool MayaW476_GetCounters(MayaW476_Counters *out);

/* Join or host the network a host stored over USB (Neuro Studio's Wi-Fi
 * settings, or harness/bench/n6bench.py wifiset). Open settings are refused;
 * hosting with no key uses MayaW476_DevicePassword(). WL_NO_SSID_AVAIL with
 * nothing stored. A stored hostname applies unless MayaW476_SetHostname() set
 * one. */
wl_status_t    MayaW476_BeginStored(void);
wl_stored_mode MayaW476_StoredMode(void);
const char    *MayaW476_StoredSSID(void);    /* "" when nothing is stored */

/* Leave the network, or stop hosting one; the radio stays on. */
void MayaW476_Disconnect(void);

/* MayaW476_Disconnect(), then the Wi-Fi radio off (deep sleep). BLE keeps
 * working. A begin turns the radio back on. */
void MayaW476_End(void);

/* Current status; cheap (reads flags). */
wl_status_t MayaW476_Status(void);

/* Addressing */

/* All zero until DHCP completes or the SoftAP is up. */
uint32_t MayaW476_LocalIP(void);
uint32_t MayaW476_SubnetMask(void);
uint32_t MayaW476_GatewayIP(void);
uint32_t MayaW476_DnsIP(int n);                  /* n = 0 or 1 */

/* A fixed address instead of DHCP; call before a begin. Before
 * MayaW476_BeginAP() it sets the access point's address and subnet. A zero
 * dns or gateway means .1 of the same /24, a zero subnet 255.255.255.0; a
 * zero local_ip goes back to DHCP and the default access point address. */
void MayaW476_Config(uint32_t local_ip, uint32_t dns, uint32_t gateway, uint32_t subnet);

/* DNS servers for MayaW476_HostByName(), in place of DHCP's. 0 for none. */
void MayaW476_SetDNS(uint32_t dns1, uint32_t dns2);

/* The name the board gives DHCP and answers to over mDNS as <name>.local.
 * Default neuron6-xxxxxx, from the chip's unique ID. Letters, digits and
 * hyphens, up to 32; call before a begin. */
void        MayaW476_SetHostname(const char *name);
const char *MayaW476_GetHostname(void);

/* Dotted-quad form of MayaW476_LocalIP(); "0.0.0.0" until there is one. */
const char *MayaW476_Ipv4String(void);

/* The link */

/* The joined SSID (STA) or the hosted one (SoftAP); "" before a begin. */
const char *MayaW476_SSID(void);

/* Signal strength of the joined network in dBm; 0 when not joined. */
int32_t MayaW476_RSSI(void);

/* The radio firmware's version string; "" before the radio has started. */
const char *MayaW476_FirmwareVersion(void);

/* Transmit power the firmware accepted, in dBm; -1 if it will not report it.
 * The board caps it at 8 dBm, the EMC-tested level, when the radio starts. */
int32_t MayaW476_TxPower(void);

/* RSSI (dBm) and SNR (dB) averaged over the last several packets; steadier
 * than MayaW476_RSSI() for a video link. Either pointer may be NULL. True
 * while joined and the firmware answered. */
bool MayaW476_AverageSignal(int32_t *rssi, int32_t *snr);

/* The 6-byte MAC of the joined access point (STA) or of this board's access
 * point (SoftAP), written to `bssid` and returned. Zeroes if none. */
uint8_t *MayaW476_BSSID(uint8_t *bssid);

/* Channel of the joined or hosted network; 0 if none. */
uint8_t MayaW476_Channel(void);

/* This board's 6-byte MAC on the interface in use, written to `mac` and
 * returned. */
uint8_t *MayaW476_MacAddress(uint8_t *mac);

/* Scanning */

/* Scan: the number of networks (up to 32, strongest first) or -1. Starts the
 * radio if no begin has run. Results stay until the next scan; item i: */
int8_t      MayaW476_ScanNetworks(void);
const char *MayaW476_ScanSSID(uint8_t i);
int32_t     MayaW476_ScanRSSI(uint8_t i);
uint8_t     MayaW476_ScanEncryption(uint8_t i);   /* wl_enc_type */
uint8_t    *MayaW476_ScanBSSID(uint8_t i, uint8_t *bssid);
uint8_t     MayaW476_ScanChannel(uint8_t i);

/* Services */

/* Resolve a host name (or a dotted quad) to an address. 1 on success. */
int MayaW476_HostByName(const char *host, uint32_t *ip);

/* Seconds since 1970-01-01 UTC from NTP, or 0 if no server answered. Asks at
 * most once an hour and counts the tick in between. Server: pool.ntp.org
 * unless MayaW476_SetTimeServer() names another (a name or a dotted quad).
 * SNTP is unauthenticated: an attacker on the path can set the time. */
uint32_t MayaW476_GetTime(void);
void     MayaW476_SetTimeServer(const char *server);

/* IEEE power save for a joined station: lower power, more latency. True if
 * the module took it; an access point cannot sleep. */
bool MayaW476_LowPowerMode(void);
bool MayaW476_NoLowPowerMode(void);

/* Key (8 to 64 characters) a host must prove before it gets the stream or
 * sends commands over Wi-Fi. Off unless set here or stored with the Wi-Fi
 * settings; "" goes back to the stored key. Applies to the next host that
 * connects. It does not encrypt the stream. False, changing nothing, for a
 * key of the wrong length. */
bool MayaW476_SetLinkKey(const char *key);

/* Settings */

/* Connect timeout for the begin functions, in milliseconds. Default 7000. */
void MayaW476_SetTimeout(uint32_t ms);

/* TCP port of the Neuro host server, and the UDP port of discovery probes.
 * Default 5000; call before MayaW476_Start(). */
void MayaW476_SetTcpPort(uint16_t port);

/* Hooks for the host server
 * MayaW476_Start(): radio, firmware, connection manager, the board's own
 *   access point (MayaW476_BeginAP()'s, else the application's settings, else
 *   the stored ones) and the host server on it. 0 on success; -3 for
 *   TRANSPORT_WIFI_STA or a joined network (access point only).
 * MayaW476_RadioStart(): module power and firmware, no network (what BLE
 *   needs). 0 once it runs.
 * MayaW476_GetIPv4_BE() / MayaW476_GetIPv4_Port(): the IP announce's address
 *   (zero until the board's access point is up) and port.
 * MayaW476_SoftApAnnounce(): the META_BOARD_SOFTAP value { name length, name,
 *   passphrase length, passphrase } and its length; 0 when not hosting. It
 *   carries the passphrase: USB link only. */
int      MayaW476_Start(int transport_mode);
int      MayaW476_RadioStart(void);
uint32_t MayaW476_GetIPv4_BE(void);
uint16_t MayaW476_GetIPv4_Port(void);
uint32_t MayaW476_SoftApAnnounce(uint8_t *out, uint32_t cap);

/* Supplied by the application: print one console message. */
void NeuroN6_DbgPrint(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W476_H */
