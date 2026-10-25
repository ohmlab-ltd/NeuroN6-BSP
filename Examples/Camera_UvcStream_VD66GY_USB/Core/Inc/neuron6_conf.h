/**
  ******************************************************************************
  * @file    neuron6_conf_template.h
  * @brief   Neuro N6 BSP configuration template.
  *
  *          Copy this file into your application as `neuron6_conf.h` and edit
  *          it there: every BSP header includes "neuron6_conf.h". The template
  *          itself is never included.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  ******************************************************************************
  */

#ifndef NEURON6_CONF_H
#define NEURON6_CONF_H

/* The BSP reaches the HAL only through this include. */
#include "stm32n6xx_hal.h"

/* Interrupt priorities (0 = highest). The defaults leave room above them for
 * an RTOS, apart from the watchdog's early warning, which must preempt
 * whatever stopped the refresh (see BSP_WDG_InitEWI). The capture paths share
 * 5 so none starves another; the header pins, tone and button sit lowest, so
 * an application callback never delays a frame or an audio buffer. */
#define BSP_BUTTON_IT_PRIORITY          14U
#define BSP_CAMERA_IT_PRIORITY          5U
#define BSP_AUDIO_OUT_IT_PRIORITY       6U
#define BSP_AUDIO_IN_IT_PRIORITY        6U
#define BSP_NPU_IT_PRIORITY             5U
#define BSP_COM_IT_PRIORITY             5U
#define BSP_THERMAL_IT_PRIORITY         5U
#define BSP_WIFI_IT_PRIORITY            5U
#define BSP_BT_IT_PRIORITY              6U
#define BSP_IO_IT_PRIORITY              8U
#define BSP_TONE_IT_PRIORITY            8U

/* Feature switches */

/* Watchdog early-wakeup interrupt support (BSP_WDG_EWI_Callback). */
#define USE_BSP_WDG_EWI                 1U

/* USB CDC console (neuron6_com). Needs the TinyUSB device stack and the
 * tusb_config.h in Utilities/usb/, which follows BSP_USE_FREERTOS. */
#define USE_BSP_COM_FEATURE             1U

/* Hardware flow control on the BLE link. One of its pins is the thermal
 * camera's power line, so a board on the thermal carrier that wants BLE sets
 * 0, and the link stays at 115200 baud. */
#define USE_BSP_BT_FLOW_CONTROL         1U

/* Let BSP_THERMAL_GetFrame restart a stalled video stream by itself. With
 * this at 0 it reports the stall instead and the application decides. */
#define BSP_THERMAL_AUTO_RESYNC         1U

/* Cache maintenance: the BSP cleans and invalidates the D-cache around every
 * DMA transfer it owns (camera frames, audio buffers, LCD framebuffers,
 * thermal video). Set 0 only if the D-cache is off or every BSP buffer sits
 * in a non-cacheable MPU region. */
#define USE_BSP_CPU_CACHE_MAINTENANCE   1U

/* Buffer placement: the BSP never allocates image or audio memory; the
 * application passes buffers in. */

/* The thermal module's own DMA descriptor MUST land in a region the linker
 * script and MPU make non-cacheable: .axisram_nc in this repository's linker
 * scripts and MPU set-up. */
#define BSP_THERMAL_DMA_NODE_SECTION    ".axisram_nc"

/* GPDMA1 channels: three BSP modules claim one each and none may share one.
 * Plain decimal literals: the modules paste them into symbol names. */
#define BSP_AUDIO_IN_DMA_CHANNEL        0
#define BSP_THERMAL_DMA_CHANNEL         1
#define BSP_AUDIO_OUT_DMA_CHANNEL       2

/* I2C: bus 1, the on-board sensor bus, stays below 400 kHz, the thermal
 * camera's CCI limit; the shipped timing gives about 295 kHz. */
/* #define BUS_I2C1_TIMING_OVERRIDE     0x10707CBBU */
/* #define BUS_I2C2_FREQUENCY           100000U */

/* Header SPI bus: the clock it comes up at. A driver sets its own per device
 * with BSP_SPI2_SetConfig. */
/* #define BUS_SPI2_FREQUENCY           4000000U */

/* Analogue input: divider from HCLK to the converter clock, and the sampling
 * window. */
/* #define BSP_ADC_CLOCK_DIVIDER        16U */
/* #define BSP_ADC_SAMPLING_TIME        ADC_SAMPLETIME_246CYCLES_5 */

/* Analogue output: modulation frequency (Hz) and duty resolution (bits). */
/* #define BSP_PWM_DEFAULT_FREQUENCY    1000U */
/* #define BSP_PWM_DEFAULT_RESOLUTION   8U */

/* Image sensor: the OV5640 by default; another is chosen at run time with
 * BSP_CAMERA_RegisterSensor. A VD66GY board calls BSP_CAMERA_VD66GY_Register
 * before BSP_CAMERA_Init and compiles neuron6_camera_vd66gy.c and
 * Components/vd6g/vd6g.c. EXT_CLOCK_HZ: the oscillator its carrier feeds it,
 * which every frame rate derives from. */
/* #define BSP_CAMERA_VD66GY_EXT_CLOCK_HZ  12000000U */

/* Camera carrier */

/* Which carrier the image sensor sits on, and so how its data pair is wired:
 * the display carrier crosses it. A wrong setting gives no error, only no
 * frames. An application for both carriers calls BSP_CAMERA_SetCarrier. */
/* #define BSP_CAMERA_DEFAULT_CARRIER   BSP_CAMERA_CARRIER_INVERTED */

/* Motion sensor: the output rates (Hz) and full scales (g, dps) that
 * BSP_MOTION_SENSOR_Init applies. */
/* #define MOTION_SENSOR_ACC_DEFAULT_ODR    104.0f */
/* #define MOTION_SENSOR_GYRO_DEFAULT_ODR   104.0f */
/* #define MOTION_SENSOR_ACC_DEFAULT_FS     2 */
/* #define MOTION_SENSOR_GYRO_DEFAULT_FS    245 */

/* Radio module */

/* Where the radio firmware image lives in memory-mapped flash (fixed by the
 * boot contract). */
/* #define BSP_WIFI_FIRMWARE_BASE       0x91E00000U */

/* SDIO dividers for the radio: enumeration at 446 kHz (the card specification
 * asks for 400 kHz) and transfers at 8.163 MHz, the clock the radio passed
 * radiated emissions with. The transfer divider must stay odd (see
 * neuron6_wifi.h): do not change it without testing emissions again. */
/* #define BSP_WIFI_SDIO_INIT_DIV       0x80U */
/* #define BSP_WIFI_SDIO_TRANSFER_DIV   0x07U */

/* USB video: at 1 the board enumerates as a UVC camera (no host driver) and
 * a console, in place of the vendor streaming interface. The application
 * supplies the frames; see Examples/Camera_UvcStream_USB. */
#define BSP_USB_UVC                     1U

/* The picture the USB descriptors (Utilities/usb/usb_descriptors.c) declare
 * and main.c runs the camera at. Width and height must be the sensor mode's
 * (main.c checks); this mode reaches 88 frames a second. */
#define UVC_FRAME_WIDTH                 640
#define UVC_FRAME_HEIGHT                480
#define UVC_FRAME_RATE                  60

/* FreeRTOS: 1 under the kernel (see neuron6_rtos.h); 0, the default, is bare
 * metal: every module runs from the main loop, waits by polling and takes no
 * lock. */
#ifndef BSP_USE_FREERTOS
#define BSP_USE_FREERTOS                0U
#endif

#endif /* NEURON6_CONF_H */
