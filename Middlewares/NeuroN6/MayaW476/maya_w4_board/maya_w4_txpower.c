/**
 ******************************************************************************
 * @file    maya_w4_txpower.c
 * @brief   Regulatory TX-power cap. See maya_w4_txpower.h.
 *
 * The TRPC limit table (wlan_set_txpwrlimit) caps STA and SoftAP alike;
 * wlan_set_sta_tx_power() alone bounds station mode only. It is layered over
 * the driver's regional table, which must not be hand-edited.
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

#include "maya_w4_txpower.h"

#include <stdio.h>
#include <string.h>

#include "wlan.h"
#include "wifi.h"

extern void NeuroN6_DbgPrint(const char *msg);

/* Modulation groups in this build's TRPC entry: 20 with CONFIG_11AX, 16 with
 * CONFIG_11AC, else 10. */
#define MAYA_W4_NUM_MOD_GRPS \
    ((t_u8)(sizeof(((wifi_txpwrlimit_config_t *)0)->txpwrlimit_entry) / \
            sizeof(wifi_txpwrlimit_entry_t)))

/* 2.4 GHz channels 1..14; the region lock still limits which are used. */
#define MAYA_W4_NUM_2G4_CHANS  14u

static wifi_txpwrlimit_t s_txpwrlimit_2g4;

/* One power value over every channel and modulation group. */
static void maya_w4_build_2g4_table(void)
{
    const t_u8 n_grps = MAYA_W4_NUM_MOD_GRPS;

    (void)memset(&s_txpwrlimit_2g4, 0, sizeof(s_txpwrlimit_2g4));
    s_txpwrlimit_2g4.subband   = (wifi_SubBand_t)0x00;   /* 2.4 GHz */
    s_txpwrlimit_2g4.num_chans = (t_u8)MAYA_W4_NUM_2G4_CHANS;

    for (t_u8 ch = 0U; ch < (t_u8)MAYA_W4_NUM_2G4_CHANS; ch++)
    {
        wifi_txpwrlimit_config_t *cfg = &s_txpwrlimit_2g4.txpwrlimit_config[ch];

        cfg->num_mod_grps         = n_grps;
        cfg->chan_desc.start_freq = 2407U;
        cfg->chan_desc.chan_width = 20U;
        cfg->chan_desc.chan_num   = (t_u8)(ch + 1U);

        for (t_u8 g = 0U; g < n_grps; g++)
        {
            cfg->txpwrlimit_entry[g].mod_group = g;
            cfg->txpwrlimit_entry[g].tx_power  = (t_u8)MAYA_W4_TX_POWER_DBM;
        }
    }
}

void maya_w4_apply_tx_power_cap(void)
{
    char dbg[176];

    maya_w4_build_2g4_table();

    /* The regulatory ceiling: per-channel TRPC limits (STA and uAP). */
    int rc = wlan_set_txpwrlimit(&s_txpwrlimit_2g4);
    if (rc != WM_SUCCESS)
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] TX POWER CAP FAILED: wlan_set_txpwrlimit rc=%d - "
                 "radio may transmit above %u dBm. DO NOT "
                 "SHIP THIS STATE.\r\n", rc, (unsigned)MAYA_W4_TX_POWER_DBM);
        NeuroN6_DbgPrint(dbg);
    }

    /* The station ceiling too, in case a firmware applies the two limits
     * independently. */
    (void)wlan_set_sta_tx_power((t_u32)MAYA_W4_TX_POWER_DBM);

    /* Log the limit the firmware reports, not the value requested. */
    {
        static wifi_txpwrlimit_t readback;   /* static: ~1 KB, keep it off the task stack */
        (void)memset(&readback, 0, sizeof(readback));

        if (wlan_get_txpwrlimit((wifi_SubBand_t)0x00, &readback) == WM_SUCCESS &&
            readback.num_chans > 0U && readback.txpwrlimit_config[0].num_mod_grps > 0U)
        {
            t_u8 worst = 0U;
            for (t_u8 c = 0U; c < readback.num_chans &&
                              c < (t_u8)MAYA_W4_NUM_2G4_CHANS; c++)
            {
                const wifi_txpwrlimit_config_t *cfg = &readback.txpwrlimit_config[c];
                for (t_u8 g = 0U; g < cfg->num_mod_grps && g < MAYA_W4_NUM_MOD_GRPS; g++)
                {
                    if (cfg->txpwrlimit_entry[g].tx_power > worst)
                    {
                        worst = cfg->txpwrlimit_entry[g].tx_power;
                    }
                }
            }

            snprintf(dbg, sizeof(dbg),
                     "[MayaW476] TX power cap: requested %u dBm, firmware "
                     "reports max %u dBm over %u ch x %u mod-groups%s\r\n",
                     (unsigned)MAYA_W4_TX_POWER_DBM, (unsigned)worst,
                     (unsigned)readback.num_chans,
                     (unsigned)readback.txpwrlimit_config[0].num_mod_grps,
                     (worst <= (t_u8)MAYA_W4_TX_POWER_DBM) ? " [OK]"
                                                           : " [OVER LIMIT!]");
            NeuroN6_DbgPrint(dbg);
        }
        else
        {
            snprintf(dbg, sizeof(dbg),
                     "[MayaW476] TX power cap applied (%u dBm) - read-back "
                     "unavailable on this firmware.\r\n",
                     (unsigned)MAYA_W4_TX_POWER_DBM);
            NeuroN6_DbgPrint(dbg);
        }
    }
}
