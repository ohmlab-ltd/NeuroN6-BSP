/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

// RHPort max operational speed can defined by board.mk
#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_DEFAULT_SPEED
#endif

#define BOARD_DEVICE_RHPORT_NUM   0
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by board.mk
#define CFG_TUSB_MCU          OPT_MCU_STM32N6
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

/* FreeRTOS OSAL: the CDC TX FIFO has two task producers plus the ISR consumer,
 * so it needs the real FIFO mutexes and atomic endpoint claims the OSAL adds.
 * Constraints that come with this setting:
 *  - tud_task() blocks on the event queue, so call sites use the non-blocking
 *    tud_task_ext(0, false) instead;
 *  - osal_freertos.h clamps waits to 0 before the scheduler starts, because
 *    this core enumerates during the pre-scheduler boot pump;
 *  - the USB IRQ priority must stay >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 *    (it is 5 == the limit) for the FromISR queue posts to be legal. */
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_FREERTOS
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED       1

/* DWC2 internal-DMA device mode on USB1_OTG_HS: the controller masters every
 * packet, which avoids the TX-FIFO-empty interrupt storm the slave path
 * produces under heavy concurrent IRQ load. Related settings in tusb_option.h
 * and tusb_mcu.h:
 *   - CFG_TUD_DWC2_SLAVE_ENABLE stays 1, so the slave path is still compiled and
 *     the mode is chosen at runtime from GHWCFG2.arch: internal DMA where the
 *     controller reports it, the slave path otherwise;
 *   - CFG_TUD_MEM_DCACHE_ENABLE is 1, so the D-cache is cleaned and invalidated
 *     around each DMA buffer. Every endpoint buffer must therefore occupy whole
 *     32-byte cache lines; TUD_EPBUF_DEF guarantees that, and a raw buffer that
 *     did not would let an invalidate evict a neighbouring variable. */
#define CFG_TUD_DWC2_DMA_ENABLE   1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED     BOARD_TUD_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#if CFG_TUD_DWC2_DMA_ENABLE
/* The Cortex-M55 D-cache line is 32 bytes: every USB DMA buffer must be aligned
 * to a whole line so a clean/invalidate touches only that buffer's lines. */
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(32)))
#else
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif
#endif
#undef  BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_HIGH_SPEED

#undef  CFG_TUD_MAX_SPEED
#define CFG_TUD_MAX_SPEED     OPT_MODE_HIGH_SPEED
//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC               1
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            1
#define CFG_TUD_VIDEO			  0
#define CFG_TUD_VIDEO_STREAMING	  0
#define CFG_TUD_VIDEO_STREAMING_BULK 0
// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 16384 : 64)
#define CFG_TUD_CDC_TX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 16384 : 64)
#define CFG_TUD_VENDOR_TX_BUFSIZE  (32*1024)
#define CFG_TUD_VENDOR_RX_BUFSIZE  (1*1024)
/* How much goes per transfer on the vendor interface. With the controller's
 * DMA the class copies its FIFO into this buffer and sends it as one
 * transfer, and each transfer is a round trip through the USB task. At the
 * class's default of 64 bytes the interface moves under 1 MB/s, too slow
 * for 720p (31 KB frames) at 30 fps; at 8 KB it holds 30 fps with the
 * network running. */
#define CFG_TUD_VENDOR_EPSIZE      (8*1024)
// CDC Endpoint transfer buffer size, more is faster
#define CFG_TUD_CDC_EP_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE  (TUD_OPT_HIGH_SPEED ? 512 : 64)

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
