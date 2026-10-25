/**
  * @file    neuron6_lcd.h
  * @brief   LCD (LTDC) support for the Ohm Lab Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_LCD_H
#define NEURON6_LCD_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One 800x480 panel, wired RGB565. Its GT911 touch controller is not driven here.
 * The BSP allocates no framebuffer: 16-byte aligned, and non-cacheable or with
 * USE_BSP_CPU_CACHE_MAINTENANCE (docs/BOOT-CONTRACT.md).
 * Changes reload per layer (the HAL masks both layers from LTDC_SRCR) at vertical
 * blanking, or at once while the display is off.
 * No LTDC interrupt handler is installed, but HAL_LTDC_Init() enables the error
 * sources: an application that enables LTDC_IRQn or LTDC_ER_IRQn needs its own. */

#define BSP_LCD_INSTANCES_NBR           1U
#define BSP_LCD_INSTANCE_0              0U

/** @brief Panel size in pixels, fixed by the hardware. */
#define BSP_LCD_WIDTH                   800U
#define BSP_LCD_HEIGHT                  480U

#define BSP_LCD_LAYERS_NBR              2U

/** @brief Layer 0 is the background; layer 1 is composited over it. */
#define BSP_LCD_LAYER_0                 0U
#define BSP_LCD_LAYER_1                 1U

/** @brief The only layer formats accepted. */
#define BSP_LCD_PIXEL_FORMAT_RGB565     LTDC_PIXEL_FORMAT_RGB565
#define BSP_LCD_PIXEL_FORMAT_ARGB4444   LTDC_PIXEL_FORMAT_ARGB4444
#define BSP_LCD_PIXEL_FORMAT_ARGB8888   LTDC_PIXEL_FORMAT_ARGB8888

/** @brief Required framebuffer start-address alignment, in bytes. */
#define BSP_LCD_FB_ALIGNMENT            16U

/** @brief BSP_LCD_SetRefreshRate() range in Hz, and the rate BSP_LCD_Init() sets. */
#define BSP_LCD_REFRESH_RATE_MIN        30U
#define BSP_LCD_REFRESH_RATE_MAX        60U
#define BSP_LCD_REFRESH_RATE_DEFAULT    60U

/** @brief Layer settings for BSP_LCD_ConfigLayer(); X1 and Y1 are exclusive. */
typedef struct
{
  uint32_t X0;          /*!< Window start column on the panel, 0..799.            */
  uint32_t X1;          /*!< Window stop column on the panel, exclusive, 1..800.  */
  uint32_t Y0;          /*!< Window start row on the panel, 0..479.               */
  uint32_t Y1;          /*!< Window stop row on the panel, exclusive, 1..480.     */
  uint32_t PixelFormat; /*!< One of the BSP_LCD_PIXEL_FORMAT_* values.            */
  uint32_t Address;     /*!< Framebuffer: non-zero, 16-byte aligned, from the caller */
  uint8_t  Alpha;       /*!< Constant layer alpha, 0 transparent to 255 opaque.   */
} BSP_LCD_LayerConfig_t;

/** @brief BSP_LCD_GetStatus() result; the four flags are sticky LTDC_ISR error bits. */
typedef struct
{
  uint32_t Underrun;         /*!< FUIF: FIFO ran dry during active display.       */
  uint32_t UnderrunWarning;  /*!< FUWIF: FIFO reached the warning threshold.      */
  uint32_t UnderrunRotation; /*!< FURIF: FIFO ran dry in the rotation path.       */
  uint32_t TransferError;    /*!< TERRIF: AXI bus error on an LTDC read.          */
  uint32_t Enabled;          /*!< 1 if the LTDC is enabled, 0 if not. Not sticky. */
  uint32_t CurrentLine;      /*!< Line being scanned; the same in two reads while
                               *   Enabled means scanout has stalled. */
} BSP_LCD_Status_t;

int32_t BSP_LCD_Init(uint32_t Instance, uint32_t Width, uint32_t Height);
int32_t BSP_LCD_DeInit(uint32_t Instance);

int32_t BSP_LCD_ConfigLayer(uint32_t Instance, uint32_t LayerIndex,
                            const BSP_LCD_LayerConfig_t *Config);
int32_t BSP_LCD_SetLayerAddress(uint32_t Instance, uint32_t LayerIndex, uint32_t Address);
int32_t BSP_LCD_SetLayerAddressVBlank(uint32_t Instance, uint32_t LayerIndex, uint32_t Address);
int32_t BSP_LCD_IsReloadPending(uint32_t Instance, uint32_t LayerIndex, uint32_t *Pending);
int32_t BSP_LCD_SetLayerVisible(uint32_t Instance, uint32_t LayerIndex, FunctionalState State);
int32_t BSP_LCD_SetTransparency(uint32_t Instance, uint32_t LayerIndex, uint8_t Transparency);
int32_t BSP_LCD_SetActiveLayer(uint32_t Instance, uint32_t LayerIndex);

/** @brief Move or resize a layer's window, no larger than its BSP_LCD_ConfigLayer()
  * size; the drawing primitives then address the new size. */
int32_t BSP_LCD_SetLayerWindow(uint32_t Instance, uint32_t LayerIndex,
                               uint32_t Xpos, uint32_t Ypos,
                               uint32_t Width, uint32_t Height);

/** @brief Drop one colour on a layer. Color is RGB888 (0x00RRGGBB) whatever the
  * format: the blender compares expanded pixels. */
int32_t BSP_LCD_SetColorKeying(uint32_t Instance, uint32_t LayerIndex, uint32_t Color);

int32_t BSP_LCD_ResetColorKeying(uint32_t Instance, uint32_t LayerIndex);

/** @brief Commit both layers' pending changes together: Immediate != 0 reloads now
  * (tears), 0 at the next vertical blank, which BSP_LCD_IsReloadPending() reports. */
int32_t BSP_LCD_Reload(uint32_t Instance, uint32_t Immediate);
int32_t BSP_LCD_GetPixelFormat(uint32_t Instance, uint32_t *PixelFormat);

int32_t BSP_LCD_DisplayOn(uint32_t Instance);
int32_t BSP_LCD_DisplayOff(uint32_t Instance);
int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *XSize);
int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *YSize);

int32_t BSP_LCD_SetRefreshRate(uint32_t Instance, uint32_t Hz);
int32_t BSP_LCD_GetRefreshRate(uint32_t Instance, uint32_t *Hz);
int32_t BSP_LCD_GetStatus(uint32_t Instance, BSP_LCD_Status_t *Status);

int32_t BSP_LCD_WritePixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Color);
int32_t BSP_LCD_ReadPixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t *Color);
int32_t BSP_LCD_FillRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                         uint32_t Width, uint32_t Height, uint32_t Color);
int32_t BSP_LCD_FillRGBRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                            const uint8_t *pData, uint32_t Width, uint32_t Height);
int32_t BSP_LCD_DrawHLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                          uint32_t Length, uint32_t Color);

/**
  * @brief  Draw a BMP file (from its "BM" signature) into the active layer.
  * @note   16-, 24- and 32-bit BI_RGB or BI_BITFIELDS, converted to the layer's format
  *         (16-bit BI_RGB as 5-5-5, 32-bit BI_RGB opaque); compressed files are refused.
  * @note   Trusted input: the header's data offset and dimensions are used as given.
  */
int32_t BSP_LCD_DrawBitmap(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                           const uint8_t *pBmp);

int32_t BSP_LCD_DrawVLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                          uint32_t Length, uint32_t Color);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_LCD_H */
