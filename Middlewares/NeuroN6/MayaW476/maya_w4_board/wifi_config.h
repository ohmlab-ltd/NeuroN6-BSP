/**
  ******************************************************************************
  * @file    wifi_config.h
  * @brief   wifi_nxp configuration overrides.
  *
  * wifi_config_default.h includes this first and defaults the rest. The board
  * selector (WIFI_IW610_BOARD_MURATA_2LL_M2, which defines IW610) and
  * SDK_OS_FREE_RTOS come from the build's -D flags.
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

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/* Module config for the board macro: SDIO voltage, timing, ED MAC values. */
#include "wifi_bt_module_config.h"

/* REGULATORY: 2.4 GHz only, the configuration EMC tested (Ohm Lab's EMC report,
 * available on request); at 0 the driver has no 5 GHz tables at all. The build
 * also passes -DCONFIG_5GHz_SUPPORT=0, and the two change struct layouts, so
 * they must match: re-enabling 5 GHz means removing both and re-testing EMC. */
#ifndef CONFIG_5GHz_SUPPORT
#define CONFIG_5GHz_SUPPORT 0
#endif

/* RF test commands bypass the limits above; this BSP never enables them. */
#if (defined(CONFIG_RF_TEST_MODE) && (CONFIG_RF_TEST_MODE != 0)) || \
    (defined(CONFIG_WIFI_RF_TEST_MODE) && (CONFIG_WIFI_RF_TEST_MODE != 0)) || \
    (defined(CONFIG_NXP_WIFI_RF_TEST_MODE) && (CONFIG_NXP_WIFI_RF_TEST_MODE != 0))
#error "the driver's RF test mode is not part of this BSP"
#endif

/* IW610 feature flags, as in the MCUXpresso SDK wifi_cli example. Without them
 * wlan_connect can sit in WLAN_SCANNING with no scan results delivered. */
#if defined(IW610)
#define CONFIG_WMM                     1
#define CONFIG_SDIO_MULTI_PORT_TX_AGGR 1
#define CONFIG_WIFI_FEATURES           1
#define CONFIG_OFFLOAD                 1

/* Auto power-save off: at 1 the driver enables deep sleep and IEEE PS at chip
 * init, and some listen-window scheduling survives the PS-off calls. */
#define CONFIG_WIFI_AUTO_POWER_SAVE    0
#define CONFIG_WIFI_CAPA               1

/* Code-size trim: features STA/SoftAP streaming with WPA2/WPA3-Personal never
 * uses. */
#define CONFIG_ROAMING                 0   /* multi-AP roaming */
#define CONFIG_TURBO_MODE              0   /* NXP proprietary turbo bit */
#define CONFIG_CLOUD_KEEP_ALIVE        0   /* cloud persistent-connection helper */
#define CONFIG_EXT_SCAN_SUPPORT        1   /* required by the IW610 scan path - keep */
#define CONFIG_WIFI_EU_CRYPTO          0   /* firmware crypto test commands; not SAE */
#define CONFIG_11R                     0   /* Fast BSS Transition */
#define CONFIG_DRIVER_OWE              0   /* Opportunistic Wireless Encryption */
#define CONFIG_11K                     0   /* Radio Resource Management */
#define CONFIG_11V                     0   /* BSS Transition Management */

/* 802.11ac is 5 GHz only. The driver then also forces CONFIG_11AX=0 (no
 * 2.4 GHz HE); 11n is ample for the JPEG stream. */
#define CONFIG_11AC                    0
#define CONFIG_WIFI_IND_RESET          1   /* keep: in-band chip recovery on firmware hang */

/* lwIP high-performance profile; lwipopts.h overrides all of it but its 32
 * netbufs. */
#define CONFIG_NETWORK_HIGH_PERF       1

/* Unsupported here: CONFIG_AMSDU_IN_AMPDU (adapter field absent) and
 * CONFIG_TX_RX_ZERO_COPY (needs scatter-gather SDIO; HAL_SDIO_*Extended_DMA is
 * single-buffer). */

/* SDIO multi-port aggregation depth; the driver clamps it to 8 with WMM and
 * multi-port TX both on, as here. */
#undef  CONFIG_SDIO_MP_AGGR_DEF_PKT_LIMIT
#define CONFIG_SDIO_MP_AGGR_DEF_PKT_LIMIT 32

/* WMM U-APSD off: its beacon-tied service intervals only add gaps to a
 * continuous stream. */
#define CONFIG_WMM_UAPSD 0

/* SoftAP (uAP) support. Must also be passed with -D: the uAP sources test it
 * before they include this header. */
#ifndef CONFIG_NXP_WIFI_SOFTAP_SUPPORT
#define CONFIG_NXP_WIFI_SOFTAP_SUPPORT 1
#endif
#endif

/* Network stack. */

/* IPv6 off. Re-enabling also needs -DCONFIG_IPV6=1: lwIP's sources derive
 * LWIP_IPV6 from it. */
#define CONFIG_IPV6              0
#define CONFIG_MAX_IPV6_ADDRESSES 0

/* Debug output, all off. */

#define CONFIG_WLCMGR_DEBUG       0
#define CONFIG_WIFI_EXTRA_DEBUG   0
#define CONFIG_WIFI_EVENTS_DEBUG  0
#define CONFIG_WIFI_CMD_RESP_DEBUG 0
#define CONFIG_WIFI_PKT_DEBUG     0
#define CONFIG_WIFI_SCAN_DEBUG    0
#define CONFIG_WIFI_IO_INFO_DUMP  0
#define CONFIG_WIFI_IO_DEBUG      0
#define CONFIG_WIFI_IO_DUMP       0
#define CONFIG_WIFI_MEM_DEBUG     0
#define CONFIG_WIFI_AMPDU_DEBUG   0
#define CONFIG_WIFI_TIMER_DEBUG   0
#define CONFIG_WIFI_SDIO_DEBUG    0
#define CONFIG_WIFI_FW_DEBUG      0
#define CONFIG_WIFI_UAP_DEBUG     0
#define CONFIG_WPS_DEBUG          0
#define CONFIG_FW_VDLL_DEBUG      0
#define CONFIG_DHCP_SERVER_DEBUG  0
#define CONFIG_FWDNLD_IO_DEBUG    0

/* Heap diagnostics. */

#define CONFIG_HEAP_DEBUG         0
#define CONFIG_HEAP_STAT          0

#endif /* WIFI_CONFIG_H */
