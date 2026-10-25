/**
  ******************************************************************************
  * @file    stm32n6_sdio_host.c
  * @brief   STM32N6 SDIO host port for NXP wifi_nxp / IW610 (MAYA-W476).
  *
  * Implements mlan_sdio_api.h's sdio_drv_* with the same symbols as NXP's
  * mlan_sdio.c (uSDHC only), which must be left out of the build.
  * CMD53 buffers must be IDMA-reachable and D-cache coherent (see
  * sdio_drv_read/sdio_drv_write). The SDMMC IRQ handler must call
  * STM32N6_SDIO_IRQHandler().
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

#include "stm32n6_sdio_host.h"
#include "maya_w4_pinmap.h"

/* FreeRTOS directly, avoiding wifi_nxp's OSA include chain. */
#include "FreeRTOS.h"
#include "semphr.h"

/* LL SDMMC, for the identify-card sequence. */
#include "stm32n6xx_ll_sdmmc.h"

#include <string.h>

/* CMD5 OCR bits; the HAL keeps its own copies private. */
#define IW610_OCR_VDD_32_33   (1U << 20U)   /* 3.2-3.3V acceptable */
#define IW610_OCR_READY_C     (1U << 31U)   /* C = card ready */

/* sdio_drv_* as in mlan_sdio_api.h, declared here to avoid the OSA chain. */
#define WM_SUCCESS  0
#define WM_FAIL     1

extern int  sdio_drv_init(void (*cd_int)(int));
extern void sdio_drv_deinit(void);
extern int  sdio_drv_creg_read (int addr, int fn, uint32_t *resp);
extern int  sdio_drv_creg_write(int addr, int fn, uint8_t data, uint32_t *resp);
extern int  sdio_drv_read (uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp);
extern int  sdio_drv_write(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp);
extern bool sdio_get_enume_status(void);

/* Card-interrupt mask, toggled by wifi_core_task around RX windows. */
extern void sdio_enable_interrupt(void);
extern void sdio_disable_interrupt(void);

/* wifi_nxp's card-interrupt entry (posts WIFI_EVENT_SDIO). Called directly
 * from the function callback: sdio_drv_init's cd_int is always NULL. */
extern void handle_cdint(int error);


/* Module state. */

static SDIO_HandleTypeDef *s_hsdio           = NULL;
static void              (*s_cd_int_cb)(int)  = NULL;
static volatile bool      s_enumerated       = false;
static SemaphoreHandle_t   s_sdio_mutex       = NULL;

/* DMA completion: given by the HAL_SDIO_{Tx,Rx,Error}CpltCallback overrides
 * (ISR), taken under s_sdio_mutex. Status: 0, or negative on a data error. */
static SemaphoreHandle_t   s_sdio_dma_done    = NULL;
static volatile int32_t    s_sdio_dma_status  = 0;

/* CACHE: the IDMA bypasses the D-cache and wifi_nxp's buffers are cacheable:
 * clean before TX, invalidate after RX. An invalidate discards the CPU's
 * unwritten data in whole 32-byte lines, so an RX buffer that does not start
 * and end on a line boundary goes through s_sdio_bounce (under the mutex). */
#define SDIO_DCACHE_LINE   32U
#define SDIO_BOUNCE_SIZE   512U
static uint8_t s_sdio_bounce[SDIO_BOUNCE_SIZE] __attribute__((aligned(SDIO_DCACHE_LINE)));

/* After a DMA error the HAL leaves State BUSY with IDMA on, so every later DMA
 * call returns HAL_BUSY: redo DATAEND's cleanup. Call with s_sdio_mutex held. */
static inline void STM32N6_SDIO_ResetAfterDmaError(void)
{
    if (s_hsdio == NULL) {
        return;
    }
    __HAL_SDIO_DISABLE_IT(s_hsdio, SDMMC_IT_DATAEND  | SDMMC_IT_DCRCFAIL |
                                   SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR |
                                   SDMMC_IT_RXOVERR);
    __HAL_SDIO_CLEAR_FLAG(s_hsdio, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
                                   SDMMC_FLAG_TXUNDERR | SDMMC_FLAG_RXOVERR |
                                   SDMMC_FLAG_DATAEND);
    /* Disable IDMA and reset DCTRL (preserve SDIOEN if set). */
    s_hsdio->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
    if ((s_hsdio->Instance->DCTRL & SDMMC_DCTRL_SDIOEN) != 0U) {
        s_hsdio->Instance->DCTRL = SDMMC_DCTRL_SDIOEN;
    } else {
        s_hsdio->Instance->DCTRL = 0U;
    }
    __SDMMC_CMDTRANS_DISABLE(s_hsdio->Instance);
    /* No public abort API, so State is set directly (safe under the mutex). */
    s_hsdio->State = HAL_SDIO_STATE_READY;
    s_hsdio->Context = 0U;  /* SDIO_CONTEXT_NONE */
}

/* CMD53 counters: wifi_nxp ignores sdio_drv_write's result, so a failed
 * firmware chunk is otherwise invisible. */
volatile uint32_t sdio_cmd53_write_ok    = 0;
volatile uint32_t sdio_cmd53_write_fail  = 0;
volatile uint32_t sdio_cmd53_read_ok     = 0;
volatile uint32_t sdio_cmd53_read_fail   = 0;
/* Last failed CMD53. */
volatile uint32_t sdio_cmd53_last_fail_addr   = 0;
volatile uint32_t sdio_cmd53_last_fail_bcnt   = 0;
volatile uint32_t sdio_cmd53_last_fail_bsize  = 0;
volatile uint32_t sdio_cmd53_last_fail_hal_err = 0;

/* IRQ counters. A runaway g_sdio_irq_count is an IRQ storm; the STA/MASK
 * snapshots show the stuck bit. The two handle_cdint counts should match. */
volatile uint32_t g_sdio_irq_count                = 0;
volatile uint32_t g_sdio_irq_sta_last             = 0;
volatile uint32_t g_sdio_irq_mask_last            = 0;
volatile uint32_t g_sdio_irq_sta_or               = 0;
volatile uint32_t g_sdio_fn_cb_count              = 0;
volatile uint32_t g_sdio_fn_cb_func1_count        = 0;
volatile uint32_t g_sdio_fn_cb_handle_cdint_count = 0;
volatile uint32_t g_sdio_handle_cdint_count       = 0;

/* Firmware-download telemetry: sdio_drv_write's progress counters, and the
 * registers STM32N6_SDIO_FwProbe() captures after wlan_init fails:
 *   fw_status_raw_e8/e9     0xDC/0xFE once firmware runs (FIRMWARE_READY,
 *                           0xFEDC); 0/0: the bootloader never handed over
 *   card_to_host_event_5c   bits 0 and 3 set: the chip accepts a chunk
 *   card_config_2_1_d9,
 *   cmd_config_0_c4         0: the ioport setup never ran or was lost
 *   fn1_block_size_lo/hi    0x00/0x01 (256 B) after download prep; 8 B: no
 *                           prep, and chunks CRC-fail
 *   card_revision_c8        non-zero: chip alive at the CCCR layer
 */
volatile uint32_t sdio_fwdnld_writes_to_memport    = 0;
volatile uint32_t sdio_fwdnld_total_bytes_to_memport = 0;
volatile uint32_t sdio_fwdnld_last_addr  = 0;
volatile uint32_t sdio_fwdnld_last_bcnt  = 0;
volatile uint32_t sdio_fwdnld_last_bsize = 0;

/* First CMD53 writes to MEM_PORT: size, leading bytes (chunk 0 is the image
 * header) and the chip's registers after each. A chunk with a bad signature
 * shows next-chunk size 0 with firmware status 0. */
#define SDIO_FWDNLD_CAPTURE_N 8
volatile uint32_t sdio_fwdnld_chunk_size[SDIO_FWDNLD_CAPTURE_N]       = {0};
volatile uint8_t  sdio_fwdnld_chunk_header[SDIO_FWDNLD_CAPTURE_N][64] = {{0}};
volatile uint8_t  sdio_fwdnld_event5c_after[SDIO_FWDNLD_CAPTURE_N]    = {0};
volatile uint16_t sdio_fwdnld_basereg_before[SDIO_FWDNLD_CAPTURE_N]   = {0};  /* what chip wanted BEFORE write */
volatile uint16_t sdio_fwdnld_basereg_after[SDIO_FWDNLD_CAPTURE_N]    = {0};  /* what chip wants AFTER write  */
volatile uint16_t sdio_fwdnld_fwstatus_after[SDIO_FWDNLD_CAPTURE_N]   = {0};
volatile uint32_t sdio_fwdnld_resp_after[SDIO_FWDNLD_CAPTURE_N]       = {0};

volatile uint8_t  sdio_fwprobe_card_to_host_event_5c = 0xFF;
volatile uint8_t  sdio_fwprobe_fw_status_raw_e8      = 0xFF;
volatile uint8_t  sdio_fwprobe_fw_status_raw_e9      = 0xFF;
volatile uint8_t  sdio_fwprobe_card_config_2_1_d9    = 0xFF;
volatile uint8_t  sdio_fwprobe_cmd_config_0_c4       = 0xFF;
volatile uint8_t  sdio_fwprobe_cmd_config_1_c5       = 0xFF;
volatile uint8_t  sdio_fwprobe_card_misc_cfg_d8      = 0xFF;
volatile uint8_t  sdio_fwprobe_fw_reset_ee           = 0xFF;
volatile uint8_t  sdio_fwprobe_card_revision_c8      = 0xFF;
volatile uint8_t  sdio_fwprobe_host_int_status_0c    = 0xFF;
volatile uint8_t  sdio_fwprobe_host_int_rsr_04       = 0xFF;
volatile uint8_t  sdio_fwprobe_host_int_mask_08      = 0xFF;
volatile uint8_t  sdio_fwprobe_fn1_block_size_lo     = 0xFF;
volatile uint8_t  sdio_fwprobe_fn1_block_size_hi     = 0xFF;
volatile uint8_t  sdio_fwprobe_read_base_0_f8        = 0xFF;
volatile uint8_t  sdio_fwprobe_read_base_1_f9        = 0xFF;
volatile uint16_t sdio_fwprobe_read_base_combined    = 0xFFFF;
volatile uint16_t sdio_fwprobe_fw_status_combined    = 0xFFFF;
volatile uint8_t  sdio_fwprobe_done                  = 0;
volatile uint32_t sdio_fwprobe_iter                  = 0;

/* More chip-state registers. */
volatile uint8_t  sdio_fwprobe_host_to_card_event_00 = 0xFF;
volatile uint8_t  sdio_fwprobe_rd_bitmap_l_10        = 0xFF;
volatile uint8_t  sdio_fwprobe_rd_bitmap_u_11        = 0xFF;
volatile uint8_t  sdio_fwprobe_wr_bitmap_l_14        = 0xFF;
volatile uint8_t  sdio_fwprobe_wr_bitmap_u_15        = 0xFF;
volatile uint8_t  sdio_fwprobe_io_port_0_e4          = 0xFF;
volatile uint8_t  sdio_fwprobe_io_port_1_e5          = 0xFF;
volatile uint8_t  sdio_fwprobe_io_port_2_e6          = 0xFF;
volatile uint8_t  sdio_fwprobe_card_ocr_0_d4         = 0xFF;
volatile uint8_t  sdio_fwprobe_card_ocr_1_d5         = 0xFF;
volatile uint8_t  sdio_fwprobe_card_ocr_3_d6         = 0xFF;
volatile uint8_t  sdio_fwprobe_card_config_d7        = 0xFF;
volatile uint8_t  sdio_fwprobe_card_rx_len_ea        = 0xFF;
volatile uint8_t  sdio_fwprobe_card_rx_unit_eb       = 0xFF;
volatile uint8_t  sdio_fwprobe_host_restart_58       = 0xFF;
volatile uint8_t  sdio_fwprobe_card_to_host_5c       = 0xFF;  /* re-read of evt5c */

/* Standard SDIO CCCR (function 0), alongside the NXP vendor registers. */
volatile uint8_t  sdio_fwprobe_cccr_revision_00      = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_sd_spec_01       = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_io_enable_02     = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_io_ready_03      = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_int_enable_04    = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_int_pending_05   = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_bus_iface_07     = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_card_cap_08      = 0xFF;
volatile uint8_t  sdio_fwprobe_cccr_high_speed_13    = 0xFF;

/* Snapshots up to 10 s after the probe: evt5c bit 0 re-asserting means the
 * chip was still verifying a chunk, not aborted. */
volatile uint8_t  sdio_fwprobe_ts_evt5c[5]    = {0xFF,0xFF,0xFF,0xFF,0xFF};   /* T=0, 100ms, 1s, 5s, 10s */
volatile uint16_t sdio_fwprobe_ts_rb[5]       = {0,0,0,0,0};
volatile uint16_t sdio_fwprobe_ts_fwstat[5]   = {0,0,0,0,0};
volatile uint32_t sdio_fwprobe_ts_tick_ms[5]  = {0,0,0,0,0};
volatile uint32_t sdio_fwprobe_ts_count       = 0;

/* Re-read after a hard PDn reset (STM32N6_SDIO_PostResetProbe). */
volatile uint8_t  sdio_fwprobe_post_reset_evt5c       = 0xFF;
volatile uint16_t sdio_fwprobe_post_reset_rb          = 0xFFFF;
volatile uint16_t sdio_fwprobe_post_reset_fwstat      = 0xFFFF;
volatile uint8_t  sdio_fwprobe_post_reset_attempted   = 0;
volatile uint8_t  sdio_fwprobe_post_reset_cccr_rev    = 0xFF;  /* chip alive at the SDIO layer? */
volatile uint8_t  sdio_fwprobe_post_reset_enum_ok     = 0;     /* 1 if CMD0/5/3/7 succeeded */
volatile uint32_t sdio_fwprobe_post_reset_enum_stage  = 0;     /* 1=CMD0, 2=CMD5, 3=CMD3, 4=CMD7 */


/* Internal helpers. */

/* Counts the real handle_cdint calls (cross-check on symbol resolution). */
static inline void s_invoke_handle_cdint(int error)
{
    g_sdio_handle_cdint_count++;
    handle_cdint(error);
}

/* Function-N interrupt from the HAL (IRQ context); only function 1 matters.
 * The chip holds DAT[1] until the task reads its status registers, so SDIOIT
 * must be masked here unconditionally, or the IRQ re-enters at once and
 * starves every task. The task path re-arms it after each transfer. */
static void STM32N6_SDIO_FunctionCallback(SDIO_HandleTypeDef *hsdio, uint32_t func)
{
    g_sdio_fn_cb_count++;

    if (func != 1U) {
        return;
    }
    g_sdio_fn_cb_func1_count++;

    __HAL_SDIO_DISABLE_IT(hsdio, SDMMC_IT_SDIOIT);

    /* STA.SDIOIT is sticky (latched on DAT[1]'s falling edge) and the HAL never
     * clears it; left set, the next re-arm fires the IRQ at once. */
    __HAL_SDIO_CLEAR_FLAG(hsdio, SDMMC_FLAG_SDIOIT);

    /* Wakes wifi_core_task. */
    g_sdio_fn_cb_handle_cdint_count++;
    s_invoke_handle_cdint(0);

    if (s_cd_int_cb != NULL) {
        s_cd_int_cb(0);
    }
}

/* DMA completion, overriding the HAL's weak callbacks (IRQ context, priority
 * 5). The error callback must clear the data-error flags, which the HAL leaves
 * latched. If both fire in one IRQ, the error status is written last, so kept. */

void HAL_SDIO_TxCpltCallback(SDIO_HandleTypeDef *hsdio)
{
    (void)hsdio;
    BaseType_t hpw = pdFALSE;
    s_sdio_dma_status = 0;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void HAL_SDIO_RxCpltCallback(SDIO_HandleTypeDef *hsdio)
{
    (void)hsdio;
    BaseType_t hpw = pdFALSE;
    s_sdio_dma_status = 0;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void HAL_SDIO_ErrorCallback(SDIO_HandleTypeDef *hsdio)
{
    BaseType_t hpw = pdFALSE;
    /* The HAL never clears these. */
    __HAL_SDIO_CLEAR_FLAG(hsdio,
        SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
        SDMMC_FLAG_RXOVERR  | SDMMC_FLAG_TXUNDERR);
    s_sdio_dma_status = -1;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

/* Re-arm SDIOIT at the end of each task-context transfer, before the bus
 * mutex is released: a CMD52/CMD53 round-trip is what releases DAT[1]. The
 * mask is read-modify-written, hence the mutex and interrupts off. */
static inline void STM32N6_SDIO_RearmIoIt(void)
{
    if (s_hsdio != NULL) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        __HAL_SDIO_ENABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
        __set_PRIMASK(primask);
    }
}

/* Public binding helpers (called from wifi_bt_config.c). */

void STM32N6_SDIO_BindHandle(SDIO_HandleTypeDef *hsdio)
{
    s_hsdio = hsdio;
}

void STM32N6_SDIO_IRQHandler(void)
{
    g_sdio_irq_count++;
    if (s_hsdio != NULL) {
        /* Snapshot first: the HAL handler changes both. */
        g_sdio_irq_sta_last  = s_hsdio->Instance->STA;
        g_sdio_irq_mask_last = s_hsdio->Instance->MASK;
        g_sdio_irq_sta_or   |= g_sdio_irq_sta_last;
        HAL_SDIO_IRQHandler(s_hsdio);
    }
}


/* wifi_nxp sdio_drv_* API implementation (contract: mlan_sdio_api.h). */

/* Wait for a command response: 1 when good (CMDSENT if none is expected), 0 on
 * timeout, -1 on CCRCFAIL (acceptable for R4, whose CRC is undefined). */
static int sdio_wait_cmd_response(bool expect_response)
{
    uint32_t tick_start = HAL_GetTick();
    uint32_t sta;
    do {
        sta = SDMMC1->STA;
        if (expect_response) {
            if (sta & SDMMC_STA_CMDREND)  return 1;
            if (sta & SDMMC_STA_CCRCFAIL) return -1;
            if (sta & SDMMC_STA_CTIMEOUT) return 0;
        } else {
            if (sta & SDMMC_STA_CMDSENT) return 1;
        }
    } while ((HAL_GetTick() - tick_start) < 1000U);
    return 0;
}

/* CMD5 (IO_SEND_OP_COND), reading R4: 1 on success, 0 on timeout. */
static int sdio_send_cmd5(uint32_t ocr_arg, uint32_t *r4_out)
{
    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = ocr_arg;
    SDMMC1->CMD = (5U << SDMMC_CMD_CMDINDEX_Pos)
                | SDMMC_CMD_WAITRESP_0      /* short response */
                | SDMMC_CMD_CPSMEN;

    int rc = sdio_wait_cmd_response(true);
    if (rc == 0) return 0;       /* timeout - chip dead */

    *r4_out = SDMMC1->RESP1;
    SDMMC1->ICR = 0xFFFFFFFFU;
    return 1;
}

/* Enumerate the IW610 by hand: HAL_SDIO_Init's CMD11 voltage-switch handshake
 * confuses it. hsdio->State is then forced to READY for HAL_SDIO_* CMD52/53. */
int sdio_drv_init(void (*cd_int)(int))
{
    if (s_hsdio == NULL) {
        return -WM_FAIL;
    }

    if (s_enumerated) {
        s_cd_int_cb = cd_int;
        return WM_SUCCESS;
    }

    if (s_sdio_mutex == NULL) {
        s_sdio_mutex = xSemaphoreCreateMutex();
        if (s_sdio_mutex == NULL) {
            return -WM_FAIL;
        }
    }

    /* DMA completion semaphore, created empty. */
    if (s_sdio_dma_done == NULL) {
        s_sdio_dma_done = xSemaphoreCreateBinary();
        if (s_sdio_dma_done == NULL) {
            vSemaphoreDelete(s_sdio_mutex);
            s_sdio_mutex = NULL;
            return -WM_FAIL;
        }
    }

    /* GPIO alternate functions and the SDMMC1 kernel clock. */
    s_hsdio->Instance = SDMMC1;
    HAL_SDIO_MspInit(s_hsdio);

    /* Identification clock, 1-bit bus, power on. SDMMC_CK = kernel /
     * (2 x CLKDIV), the kernel clock from PLL1 as the FSBL set it: 114.29 MHz /
     * (2 x 143) = 399.6 kHz, within the spec's 400 kHz. Odd divider: see the
     * transfer clock below. */
    #define MAYA_W4_SDIO_INIT_CLKDIV   (0x8FU)
    SDMMC1->CLKCR = (MAYA_W4_SDIO_INIT_CLKDIV << SDMMC_CLKCR_CLKDIV_Pos);
    SDMMC1->POWER = SDMMC_POWER_PWRCTRL;
    HAL_Delay(20U);

    /* CMD0: go idle (no response). */
    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = 0U;
    SDMMC1->CMD = (0U << SDMMC_CMD_CMDINDEX_Pos) | SDMMC_CMD_CPSMEN;
    (void)sdio_wait_cmd_response(false);
    SDMMC1->ICR = 0xFFFFFFFFU;

    /* CMD5(0) probe: needs num_io_funcs >= 1. */
    uint32_t r4 = 0U;
    if (sdio_send_cmd5(0U, &r4) != 1) {
        return -WM_FAIL;
    }
    uint8_t num_io_funcs = (uint8_t)((r4 >> 28) & 0x07U);
    if (num_io_funcs == 0U) {
        return -WM_FAIL;
    }

    /* CMD5 OCR vote: poll the C bit until ready. */
    uint32_t poll = 1000U;
    while (poll-- > 0U) {
        if (sdio_send_cmd5(IW610_OCR_VDD_32_33, &r4) != 1) {
            return -WM_FAIL;
        }
        if ((r4 & IW610_OCR_READY_C) != 0U) {
            break;
        }
        HAL_Delay(1U);
    }
    if ((r4 & IW610_OCR_READY_C) == 0U) {
        return -WM_FAIL;
    }

    /* CMD3: get the RCA. ILLEGAL_CMD is allowed briefly after CMD0: retry. */
    uint16_t rca = 1U;
    uint32_t err;
    poll = 1000U;
    do {
        err = SDMMC_CmdSetRelAdd(SDMMC1, &rca);
        if (err == HAL_SDIO_ERROR_NONE) break;
        if (err != SDMMC_ERROR_ILLEGAL_CMD) {
            return -WM_FAIL;
        }
        HAL_Delay(1U);
    } while (poll-- > 0U);
    if (err != HAL_SDIO_ERROR_NONE) {
        return -WM_FAIL;
    }

    /* CMD7: select the card; CMD52/CMD53 work from here. */
    if (SDMMC_CmdSelDesel(SDMMC1, ((uint32_t)rca) << 16U) != HAL_SDIO_ERROR_NONE) {
        return -WM_FAIL;
    }

    /* Forge the HAL state so HAL_SDIO_* work. */
    s_hsdio->State     = HAL_SDIO_STATE_READY;
    s_hsdio->ErrorCode = HAL_SDIO_ERROR_NONE;
    s_hsdio->Context   = SDIO_CONTEXT_NONE;

    /* 4-bit bus (CMD52 to CCCR and CLKCR.WIDBUS). */
    if (HAL_SDIO_SetDataBusWidth(s_hsdio, HAL_SDIO_4_WIRES_MODE) != HAL_OK) {
        return -WM_FAIL;
    }

    /* Transfer clock; read-modify-write keeps WIDBUS.
     * EMC: do not change without retesting radiated emissions. Harmonics land
     * on 200 MHz when N = IC4 x (2 x CLKDIV) is divisible by 4, so IC4
     * (maya_w4_hw.c) and CLKDIV must both be odd. IC4 = 7, CLKDIV = 7: N = 98,
     * a free-running 8.163 MHz, the tested clock (about 2.4 MB/s of stream). */
    #define MAYA_W4_SDIO_XFER_CLKDIV   (0x07U)   /* kernel/14 = 8.163 MHz */
    {
        uint32_t clkcr = SDMMC1->CLKCR;
        clkcr &= ~SDMMC_CLKCR_CLKDIV_Msk;
        clkcr |= (MAYA_W4_SDIO_XFER_CLKDIV << SDMMC_CLKCR_CLKDIV_Pos);

        /* Hardware flow control is required for writes: without it a FIFO
         * underrun sends undefined bytes under a valid CMD53 CRC. */
        clkcr |= SDMMC_CLKCR_HWFC_EN;

        /* Never set SDMMC_CLKCR_PWRSAV: the IW610's DAT1 interrupt is
         * synchronous, sampled only while SDMMC_CK runs, so clock gating hangs
         * bring-up (asynchronous interrupt mode is not implemented). */

        SDMMC1->CLKCR = clkcr;
        HAL_Delay(5U);   /* let new clock stabilise */
    }

    /* Function-1 interrupt. */
    s_cd_int_cb = cd_int;
    if (HAL_SDIO_RegisterIOFunctionCallback(s_hsdio,
                                            HAL_SDIO_FUNCTION_1,
                                            STM32N6_SDIO_FunctionCallback) != HAL_OK) {
        return -WM_FAIL;
    }
    if (HAL_SDIO_EnableIOFunction(s_hsdio, HAL_SDIO_FUNCTION_1) != HAL_OK) {
        return -WM_FAIL;
    }
    if (HAL_SDIO_EnableIOFunctionInterrupt(s_hsdio, HAL_SDIO_FUNCTION_1) != HAL_OK) {
        return -WM_FAIL;
    }

    /* The HAL calls above leave the NVIC line off. Priority 5 is >=
     * configMAX_SYSCALL_INTERRUPT_PRIORITY, so FromISR calls are legal. */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    s_enumerated = true;
    return WM_SUCCESS;
}

void sdio_drv_deinit(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        (void)HAL_SDIO_DeInit(s_hsdio);
    }
    s_enumerated = false;
    s_cd_int_cb  = NULL;
    if (s_sdio_mutex != NULL) {
        vSemaphoreDelete(s_sdio_mutex);
        s_sdio_mutex = NULL;
    }
    if (s_sdio_dma_done != NULL) {
        vSemaphoreDelete(s_sdio_dma_done);
        s_sdio_dma_done = NULL;
    }
}

/* CMD52 read: 1 on success, 0 on failure (wifi_nxp convention). */
int sdio_drv_creg_read(int addr, int fn, uint32_t *resp)
{
    if (s_hsdio == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_DirectCmd_TypeDef cmd = {
        .Reg_Addr       = (uint32_t)addr,
        .ReadAfterWrite = HAL_SDIO_WRITE_ONLY,        /* unused for read */
        .IOFunctionNbr  = (uint8_t)fn,
    };

    uint8_t value = 0U;

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    HAL_StatusTypeDef hs = HAL_SDIO_ReadDirect(s_hsdio, &cmd, &value);

    STM32N6_SDIO_RearmIoIt();

    (void)xSemaphoreGive(s_sdio_mutex);

    if (hs != HAL_OK) {
        return 0;
    }
    *resp = (uint32_t)value;
    return 1;
}

/* CMD52 write, with read-after-write. */
int sdio_drv_creg_write(int addr, int fn, uint8_t data, uint32_t *resp)
{
    if (s_hsdio == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_DirectCmd_TypeDef cmd = {
        .Reg_Addr       = (uint32_t)addr,
        .ReadAfterWrite = HAL_SDIO_READ_AFTER_WRITE,
        .IOFunctionNbr  = (uint8_t)fn,
    };

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    HAL_StatusTypeDef hs = HAL_SDIO_WriteDirect(s_hsdio, &cmd, data);

    STM32N6_SDIO_RearmIoIt();

    (void)xSemaphoreGive(s_sdio_mutex);

    if (hs != HAL_OK) {
        return 0;
    }
    *resp = (uint32_t)data;     /* the HAL does not return the RAW echo */
    return 1;
}

/* CMD53 read over DMA. buf must be DMA-reachable and 4-byte aligned (IDMABASER
 * ignores the low two bits). A buffer that is not whole cache lines goes
 * through s_sdio_bounce if it fits; a larger one must be whole lines. */
int sdio_drv_read(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp)
{
    if (s_hsdio == NULL || buf == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_ExtendedCmd_TypeDef cmd = {
        .Reg_Addr      = addr,
        /* Fixed address: MEM_PORT is a FIFO, so every byte must reach the
         * same register. */
        .OpCode        = HAL_SDIO_OP_CODE_NO_INC,
        .Block_Mode    = (bcnt > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE,
        .IOFunctionNbr = fn,
    };

    const uint32_t len = bcnt * bsize;
    uint8_t *dst = buf;
    if (((((uint32_t)buf) | len) & (SDIO_DCACHE_LINE - 1U)) != 0U && len <= SDIO_BOUNCE_SIZE) {
        dst = s_sdio_bounce;
    }

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    /* The HAL counts blocks from block_size (unset: zero blocks, and the
       transfer hangs). Set under the mutex, with its own transfer. */
    s_hsdio->block_size = (uint16_t)bsize;

    /* Drop a stale give from a transfer that finished after timing out. */
    (void)xSemaphoreTake(s_sdio_dma_done, 0);
    s_sdio_dma_status = 0;

    HAL_StatusTypeDef hs = HAL_SDIO_ReadExtended_DMA(s_hsdio, &cmd, dst, len);

    if (hs == HAL_OK) {
        /* A healthy CMD53 takes under 10 ms; 200 ms still leaves recovery of
         * a wedged chip to wifi_nxp's retry. */
        if (xSemaphoreTake(s_sdio_dma_done, pdMS_TO_TICKS(200)) != pdTRUE) {
            hs = HAL_TIMEOUT;
        } else if (s_sdio_dma_status != 0) {
            hs = HAL_ERROR;
        }
    }

    if (hs == HAL_OK) {
        /* CACHE: invalidate after RX, dropping lines prefetched meanwhile. */
        SCB_InvalidateDCache_by_Addr((volatile void *)dst, (int32_t)len);
        if (dst != buf) {
            (void)memcpy(buf, dst, len);
        }
    } else {
        STM32N6_SDIO_ResetAfterDmaError();
    }

    STM32N6_SDIO_RearmIoIt();

    (void)xSemaphoreGive(s_sdio_mutex);

    if (hs == HAL_OK) {
        sdio_cmd53_read_ok++;
    } else {
        sdio_cmd53_read_fail++;
        sdio_cmd53_last_fail_hal_err = HAL_SDIO_GetError(s_hsdio);
    }

    *resp = 0U;
    return (hs == HAL_OK) ? 1 : 0;
}

/* CMD53 write over DMA. buf must be DMA-reachable and 4-byte aligned. */
int sdio_drv_write(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp)
{
    if (s_hsdio == NULL || buf == NULL || resp == NULL) {
        return 0;
    }

    /* Telemetry: the next-chunk size the chip asks for before this write. */
    if (addr == 0x10000U) {
        uint32_t idx_pre = sdio_fwdnld_writes_to_memport;   /* not yet incremented */
        if (idx_pre < SDIO_FWDNLD_CAPTURE_N) {
            uint32_t r = 0U;
            (void)sdio_drv_creg_read(0xF8, 1, &r);
            uint16_t base = (uint16_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xF9, 1, &r);
            base |= (uint16_t)((r & 0xFFU) << 8);
            sdio_fwdnld_basereg_before[idx_pre] = base;
        }
    }

    HAL_SDIO_ExtendedCmd_TypeDef cmd = {
        .Reg_Addr      = addr,
        /* Fixed address, as in sdio_drv_read. */
        .OpCode        = HAL_SDIO_OP_CODE_NO_INC,
        .Block_Mode    = (bcnt > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE,
        .IOFunctionNbr = fn,
    };

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    /* See sdio_drv_read for the block_size note. */
    s_hsdio->block_size = (uint16_t)bsize;

    /* As in sdio_drv_read: drop a stale give, start the DMA, wait. */
    (void)xSemaphoreTake(s_sdio_dma_done, 0);
    s_sdio_dma_status = 0;

    /* CACHE: clean before TX; the IDMA bypasses the D-cache. */
    SCB_CleanDCache_by_Addr((uint32_t *)(void *)buf,
                            (int32_t)(bcnt * bsize));

    HAL_StatusTypeDef hs = HAL_SDIO_WriteExtended_DMA(s_hsdio, &cmd, buf,
                                                      bcnt * bsize);

    if (hs == HAL_OK) {
        /* 200 ms - see the matching note in sdio_drv_read. */
        if (xSemaphoreTake(s_sdio_dma_done, pdMS_TO_TICKS(200)) != pdTRUE) {
            hs = HAL_TIMEOUT;
        } else if (s_sdio_dma_status != 0) {
            hs = HAL_ERROR;
        }
    }

    if (hs != HAL_OK) {
        STM32N6_SDIO_ResetAfterDmaError();
    }

    STM32N6_SDIO_RearmIoIt();

    (void)xSemaphoreGive(s_sdio_mutex);

    /* Writes to MEM_PORT: download chunks, then TX frames. The first few also
     * snapshot the chip's event, next-chunk-size and firmware-status registers. */
    if (addr == 0x10000U) {
        uint32_t idx = sdio_fwdnld_writes_to_memport;   /* pre-increment value */
        sdio_fwdnld_writes_to_memport++;
        sdio_fwdnld_total_bytes_to_memport += (bcnt * bsize);
        sdio_fwdnld_last_addr  = addr;
        sdio_fwdnld_last_bcnt  = bcnt;
        sdio_fwdnld_last_bsize = bsize;

        if (idx < SDIO_FWDNLD_CAPTURE_N) {
            sdio_fwdnld_chunk_size[idx] = bcnt * bsize;
            /* Chunk 0 starts with the image header the ROM bootloader checks. */
            for (uint32_t k = 0U; k < 64U; k++) {
                sdio_fwdnld_chunk_header[idx][k] = buf[k];
            }
            sdio_fwdnld_resp_after[idx] = *resp;

            /* Post-write snapshot: CMD52 only, under a millisecond. */
            uint32_t r = 0U;
            (void)sdio_drv_creg_read(0x5C, 1, &r);
            sdio_fwdnld_event5c_after[idx] = (uint8_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xF8, 1, &r);
            uint16_t base = (uint16_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xF9, 1, &r);
            base |= (uint16_t)((r & 0xFFU) << 8);
            sdio_fwdnld_basereg_after[idx] = base;
            (void)sdio_drv_creg_read(0xE8, 1, &r);
            uint16_t fwst = (uint16_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xE9, 1, &r);
            fwst |= (uint16_t)((r & 0xFFU) << 8);
            sdio_fwdnld_fwstatus_after[idx] = fwst;
        }
    }

    if (hs == HAL_OK) {
        sdio_cmd53_write_ok++;
    } else {
        sdio_cmd53_write_fail++;
        sdio_cmd53_last_fail_addr     = addr;
        sdio_cmd53_last_fail_bcnt     = bcnt;
        sdio_cmd53_last_fail_bsize    = bsize;
        sdio_cmd53_last_fail_hal_err  = HAL_SDIO_GetError(s_hsdio);
    }

    *resp = 0U;
    return (hs == HAL_OK) ? 1 : 0;
}

bool sdio_get_enume_status(void)
{
    return s_enumerated;
}

/* Function-1 card interrupt, toggled by wifi_core_task around its RX polls;
 * first enabled in sdio_drv_init(). */
void sdio_enable_interrupt(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        /* Host mask, unconditionally, not HAL_SDIO_EnableIOFunctionInterrupt:
         * that returns without re-arming once CCCR4 shows the function
         * enabled (a lost wake-up with DAT[1] held), and costs a CMD52 per
         * call, hundreds of times a second. */
        __HAL_SDIO_ENABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
    }
}

void sdio_disable_interrupt(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        /* Host mask only: the HAL's disable sends CMD52s outside the mutex
         * and clears state sdio_enable_interrupt never restores, after which
         * the HAL stops dispatching the SDIO interrupt. */
        __HAL_SDIO_DISABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
    }
}

/* FreeRTOS v11 helper the driver's info dump references; a weak no-op for
 * older kernels. */
__attribute__((weak)) char *vTaskListTasks(char *pcWriteBuffer, size_t uxBufferLength)
{
    (void)uxBufferLength;
    if (pcWriteBuffer != NULL) { pcWriteBuffer[0] = '\0'; }
    return pcWriteBuffer;
}


/* See stm32n6_sdio_host.h. CMD52 only, so it works when CMD53 does not. */
void STM32N6_SDIO_FwProbe(void)
{
    uint32_t resp = 0U;

    sdio_fwprobe_iter++;

    /* CARD_TO_HOST_EVENT_REG = 0x5C. Bits 0/1/3 = DN_LD_CARD_RDY /
       UP_LD_CARD_RDY / CARD_IO_READY. */
    (void)sdio_drv_creg_read(0x5C, 1, &resp);
    sdio_fwprobe_card_to_host_event_5c = (uint8_t)(resp & 0xFFU);

    /* CARD_FW_STATUS0/1_REG = 0xE8/0xE9: FIRMWARE_READY = 0xFEDC once up. */
    (void)sdio_drv_creg_read(0xE8, 1, &resp);
    sdio_fwprobe_fw_status_raw_e8 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xE9, 1, &resp);
    sdio_fwprobe_fw_status_raw_e9 = (uint8_t)(resp & 0xFFU);
    sdio_fwprobe_fw_status_combined =
        (uint16_t)sdio_fwprobe_fw_status_raw_e8
        | ((uint16_t)sdio_fwprobe_fw_status_raw_e9 << 8);

    /* CARD_CONFIG_2_1_REG = 0xD9.  Bit 0 = CMD53_NEW_MODE. */
    (void)sdio_drv_creg_read(0xD9, 1, &resp);
    sdio_fwprobe_card_config_2_1_d9 = (uint8_t)(resp & 0xFFU);

    /* CMD_CONFIG_0 = 0xC4 (bit 2 = CMD_PORT_RD_LEN_EN). */
    (void)sdio_drv_creg_read(0xC4, 1, &resp);
    sdio_fwprobe_cmd_config_0_c4 = (uint8_t)(resp & 0xFFU);

    /* CMD_CONFIG_1 = 0xC5 (bit 0 = CMD_PORT_AUTO_EN). */
    (void)sdio_drv_creg_read(0xC5, 1, &resp);
    sdio_fwprobe_cmd_config_1_c5 = (uint8_t)(resp & 0xFFU);

    /* CARD_MISC_CFG_REG = 0xD8 (bit 4 = AUTO_RE_ENABLE_INT). */
    (void)sdio_drv_creg_read(0xD8, 1, &resp);
    sdio_fwprobe_card_misc_cfg_d8 = (uint8_t)(resp & 0xFFU);

    /* CARD_FW_RESET_REG = 0xEE: 0 in steady state. */
    (void)sdio_drv_creg_read(0xEE, 1, &resp);
    sdio_fwprobe_fw_reset_ee = (uint8_t)(resp & 0xFFU);

    /* CARD_REVISION_REG = 0xC8. Non-zero = silicon alive at CCCR layer. */
    (void)sdio_drv_creg_read(0xC8, 1, &resp);
    sdio_fwprobe_card_revision_c8 = (uint8_t)(resp & 0xFFU);

    /* HOST_INT_STATUS_REG = 0x0C: 0 once every interrupt is acknowledged; a
       pending bit can block a download restart. */
    (void)sdio_drv_creg_read(0x0C, 1, &resp);
    sdio_fwprobe_host_int_status_0c = (uint8_t)(resp & 0xFFU);

    /* HOST_INT_RSR_REG = 0x04: read-to-reset IRQ flags. */
    (void)sdio_drv_creg_read(0x04, 1, &resp);
    sdio_fwprobe_host_int_rsr_04 = (uint8_t)(resp & 0xFFU);

    /* HOST_INT_MASK_REG = 0x08. Interrupt enable bits. */
    (void)sdio_drv_creg_read(0x08, 1, &resp);
    sdio_fwprobe_host_int_mask_08 = (uint8_t)(resp & 0xFFU);

    /* FN1 block size (CCCR 0x110/0x111): 256 bytes after download prep, 8
       without. */
    (void)sdio_drv_creg_read(0x110, 0, &resp);
    sdio_fwprobe_fn1_block_size_lo = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x111, 0, &resp);
    sdio_fwprobe_fn1_block_size_hi = (uint8_t)(resp & 0xFFU);

    /* READ_BASE_0/1_REG = 0xF8/0xF9: next chunk size wanted. 0 with firmware
       status 0: the chip gave up on the download. */
    (void)sdio_drv_creg_read(0xF8, 1, &resp);
    sdio_fwprobe_read_base_0_f8 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xF9, 1, &resp);
    sdio_fwprobe_read_base_1_f9 = (uint8_t)(resp & 0xFFU);
    sdio_fwprobe_read_base_combined =
        (uint16_t)sdio_fwprobe_read_base_0_f8
        | ((uint16_t)sdio_fwprobe_read_base_1_f9 << 8);

    /* More chip-side registers. */

    /* HOST_TO_CARD_EVENT_REG = 0x00 (host-to-chip events). */
    (void)sdio_drv_creg_read(0x00, 1, &resp);
    sdio_fwprobe_host_to_card_event_00 = (uint8_t)(resp & 0xFFU);

    /* RD/WR port bitmaps (data path, once firmware is up). */
    (void)sdio_drv_creg_read(0x10, 1, &resp);
    sdio_fwprobe_rd_bitmap_l_10 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x11, 1, &resp);
    sdio_fwprobe_rd_bitmap_u_11 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x14, 1, &resp);
    sdio_fwprobe_wr_bitmap_l_14 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x15, 1, &resp);
    sdio_fwprobe_wr_bitmap_u_15 = (uint8_t)(resp & 0xFFU);

    /* IO_PORT registers (the driver hard-codes MEM_PORT instead). */
    (void)sdio_drv_creg_read(0xE4, 1, &resp);
    sdio_fwprobe_io_port_0_e4 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xE5, 1, &resp);
    sdio_fwprobe_io_port_1_e5 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xE6, 1, &resp);
    sdio_fwprobe_io_port_2_e6 = (uint8_t)(resp & 0xFFU);

    /* Chip OCR + CARD_CONFIG, which should mirror enumeration time. */
    (void)sdio_drv_creg_read(0xD4, 1, &resp);
    sdio_fwprobe_card_ocr_0_d4 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xD5, 1, &resp);
    sdio_fwprobe_card_ocr_1_d5 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xD6, 1, &resp);
    sdio_fwprobe_card_ocr_3_d6 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xD7, 1, &resp);
    sdio_fwprobe_card_config_d7 = (uint8_t)(resp & 0xFFU);

    /* Card RX length and unit: bytes to upload; 0 before firmware is up. */
    (void)sdio_drv_creg_read(0xEA, 1, &resp);
    sdio_fwprobe_card_rx_len_ea = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0xEB, 1, &resp);
    sdio_fwprobe_card_rx_unit_eb = (uint8_t)(resp & 0xFFU);

    /* HOST_RESTART_REG = 0x58: bit 2 = a download CRC error the host missed. */
    (void)sdio_drv_creg_read(0x58, 1, &resp);
    sdio_fwprobe_host_restart_58 = (uint8_t)(resp & 0xFFU);

    /* Re-read CARD_TO_HOST_EVENT_REG to check the value is stable. */
    (void)sdio_drv_creg_read(0x5C, 1, &resp);
    sdio_fwprobe_card_to_host_5c = (uint8_t)(resp & 0xFFU);

    /* Standard SDIO CCCR (function 0). */
    (void)sdio_drv_creg_read(0x00, 0, &resp);
    sdio_fwprobe_cccr_revision_00 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x01, 0, &resp);
    sdio_fwprobe_cccr_sd_spec_01 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x02, 0, &resp);
    sdio_fwprobe_cccr_io_enable_02 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x03, 0, &resp);
    sdio_fwprobe_cccr_io_ready_03 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x04, 0, &resp);
    sdio_fwprobe_cccr_int_enable_04 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x05, 0, &resp);
    sdio_fwprobe_cccr_int_pending_05 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x07, 0, &resp);
    sdio_fwprobe_cccr_bus_iface_07 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x08, 0, &resp);
    sdio_fwprobe_cccr_card_cap_08 = (uint8_t)(resp & 0xFFU);
    (void)sdio_drv_creg_read(0x13, 0, &resp);
    sdio_fwprobe_cccr_high_speed_13 = (uint8_t)(resp & 0xFFU);

    /* Snapshots over 10 s: evt5c bit 0 re-asserting means a timing problem,
       not a content one. */
    {
        const uint32_t delays_ms[5] = { 0U, 100U, 1000U, 5000U, 10000U };
        uint32_t total = 0U;
        for (uint32_t i = 0U; i < 5U; i++) {
            if (delays_ms[i] > total) {
                HAL_Delay(delays_ms[i] - total);
                total = delays_ms[i];
            }
            uint32_t r = 0U;
            (void)sdio_drv_creg_read(0x5C, 1, &r);
            sdio_fwprobe_ts_evt5c[i] = (uint8_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xF8, 1, &r);
            uint16_t rb = (uint16_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xF9, 1, &r);
            rb |= (uint16_t)((r & 0xFFU) << 8);
            sdio_fwprobe_ts_rb[i] = rb;
            (void)sdio_drv_creg_read(0xE8, 1, &r);
            uint16_t fws = (uint16_t)(r & 0xFFU);
            (void)sdio_drv_creg_read(0xE9, 1, &r);
            fws |= (uint16_t)((r & 0xFFU) << 8);
            sdio_fwprobe_ts_fwstat[i] = fws;
            sdio_fwprobe_ts_tick_ms[i] = HAL_GetTick();
            sdio_fwprobe_ts_count = i + 1U;
        }
    }

    sdio_fwprobe_done = 1U;
}


/* See stm32n6_sdio_host.h: PDn reset, re-enumerate, re-read. */
void STM32N6_SDIO_PostResetProbe(void)
{
    extern void MAYA_W4_PowerCycle(void);

    sdio_fwprobe_post_reset_attempted = 1U;
    sdio_fwprobe_post_reset_enum_stage = 0U;

    /* After POR the bootloader waits for enumeration. */
    MAYA_W4_PowerCycle();

    /* CMD52 on function 1 needs transfer state: re-enumerate. */

    /* CMD0 - go idle */
    if (SDMMC_CmdGoIdleState(s_hsdio->Instance) != HAL_SDIO_ERROR_NONE) {
        return;  /* enum_stage stays 0 - failed at CMD0 */
    }
    sdio_fwprobe_post_reset_enum_stage = 1U;

    /* CMD5 with arg=0 - probe */
    uint32_t resp4 = 0U;
    if (SDMMC_CmdSendOperationcondition(s_hsdio->Instance, 0U, &resp4) != HAL_SDIO_ERROR_NONE) {
        return;
    }

    /* CMD5 with OCR voting - poll C bit until ready (1s budget) */
    uint32_t timeout_ms = 1000U;
    do {
        if (SDMMC_CmdSendOperationcondition(s_hsdio->Instance, IW610_OCR_VDD_32_33, &resp4)
            != HAL_SDIO_ERROR_NONE) {
            return;
        }
        if ((resp4 & IW610_OCR_READY_C) != 0U) break;
        HAL_Delay(1U);
    } while (--timeout_ms);
    if (timeout_ms == 0U) return;
    sdio_fwprobe_post_reset_enum_stage = 2U;

    /* CMD3 - get RCA, with retry for ILLEGAL_CMD as per spec */
    uint16_t rca = 1U;
    timeout_ms = 1000U;
    uint32_t err;
    do {
        err = SDMMC_CmdSetRelAdd(s_hsdio->Instance, &rca);
        if (err == HAL_SDIO_ERROR_NONE) break;
        if (err != SDMMC_ERROR_ILLEGAL_CMD) return;
        HAL_Delay(1U);
    } while (--timeout_ms);
    if (timeout_ms == 0U) return;
    sdio_fwprobe_post_reset_enum_stage = 3U;

    /* CMD7 - select card */
    if (SDMMC_CmdSelDesel(s_hsdio->Instance, ((uint32_t)rca) << 16U) != HAL_SDIO_ERROR_NONE) {
        return;
    }
    sdio_fwprobe_post_reset_enum_stage = 4U;
    sdio_fwprobe_post_reset_enum_ok = 1U;

    /* Give the bootloader a moment. */
    HAL_Delay(20U);

    /* CCCR revision (fn=0) as an alive check. */
    uint32_t r = 0U;
    (void)sdio_drv_creg_read(0x00, 0, &r);
    sdio_fwprobe_post_reset_cccr_rev = (uint8_t)(r & 0xFFU);

    /* Ready for download: evt5c 0x0d, next-chunk size non-zero, status 0. */
    (void)sdio_drv_creg_read(0x5C, 1, &r);
    sdio_fwprobe_post_reset_evt5c = (uint8_t)(r & 0xFFU);
    (void)sdio_drv_creg_read(0xF8, 1, &r);
    uint16_t rb = (uint16_t)(r & 0xFFU);
    (void)sdio_drv_creg_read(0xF9, 1, &r);
    rb |= (uint16_t)((r & 0xFFU) << 8);
    sdio_fwprobe_post_reset_rb = rb;
    (void)sdio_drv_creg_read(0xE8, 1, &r);
    uint16_t fws = (uint16_t)(r & 0xFFU);
    (void)sdio_drv_creg_read(0xE9, 1, &r);
    fws |= (uint16_t)((r & 0xFFU) << 8);
    sdio_fwprobe_post_reset_fwstat = fws;
}
