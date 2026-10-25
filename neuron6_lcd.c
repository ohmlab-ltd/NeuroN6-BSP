/**
  * @file    neuron6_lcd.c
  * @brief   LCD (LTDC) support for the Ohm Lab Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_conf.h"
#include "neuron6_lcd.h"

#ifndef USE_BSP_CPU_CACHE_MAINTENANCE
#error "neuron6_conf.h must define USE_BSP_CPU_CACHE_MAINTENANCE. Copy the current neuron6_conf_template.h."
#endif

/* Timings of the fitted 800x480 panel; not configurable. */
#define LCD_HSYNC_WIDTH                 48U
#define LCD_HBP                         88U
#define LCD_HFP                         40U
#define LCD_VSYNC_HEIGHT                3U
#define LCD_VBP                         32U
#define LCD_VFP                         13U

#define LCD_HTOTAL   (LCD_HSYNC_WIDTH + LCD_HBP + BSP_LCD_WIDTH + LCD_HFP)
#define LCD_VTOTAL   (LCD_VSYNC_HEIGHT + LCD_VBP + BSP_LCD_HEIGHT + LCD_VFP)
#define LCD_PIXELS_PER_FRAME  ((uint32_t)LCD_HTOTAL * (uint32_t)LCD_VTOTAL)

_Static_assert(LCD_HTOTAL == 976U, "LTDC horizontal total changed; refresh-rate maths depends on it");
_Static_assert(LCD_VTOTAL == 528U, "LTDC vertical total changed; refresh-rate maths depends on it");

/* IC16 divider range. */
#define LCD_CLK_DIVIDER_MIN             1U
#define LCD_CLK_DIVIDER_MAX             256U

/* The LTDC error flags this module reports and clears, and their clear bits. */
#define LCD_ISR_ERROR_FLAGS  (LTDC_ISR_FUIF_Msk | LTDC_ISR_FUWIF_Msk | \
                              LTDC_ISR_FURIF_Msk | LTDC_ISR_TERRIF_Msk)
#define LCD_ICR_ERROR_FLAGS  (LTDC_ICR_CFUIF_Msk | LTDC_ICR_CFUWIF_Msk | \
                              LTDC_ICR_CFURIF_Msk | LTDC_ICR_CTERRIF_Msk)

#if (USE_BSP_CPU_CACHE_MAINTENANCE == 1)
/* Clean: push CPU writes out for the LTDC to see. */
#define LCD_CACHE_CLEAN(Addr, Size) \
  SCB_CleanDCache_by_Addr((void *)(Addr), (int32_t)(Size))
/* Clean and invalidate: the next read refetches. */
#define LCD_CACHE_CLEAN_INVALIDATE(Addr, Size) \
  SCB_CleanInvalidateDCache_by_Addr((void *)(Addr), (int32_t)(Size))
#else
#define LCD_CACHE_CLEAN(Addr, Size)            do { (void)(Addr); (void)(Size); } while (0)
#define LCD_CACHE_CLEAN_INVALIDATE(Addr, Size) do { (void)(Addr); (void)(Size); } while (0)
#endif /* USE_BSP_CPU_CACHE_MAINTENANCE */

/* Layer state, kept here: the shadowed layer registers do not reliably read back. */
typedef struct
{
  uint32_t Address;
  uint32_t PixelFormat;
  uint32_t Width;         /*!< Current window width in pixels.             */
  uint32_t Height;        /*!< Current window height in pixels.            */
  uint32_t BufWidth;      /*!< Width the framebuffer was sized for.        */
  uint32_t BufHeight;     /*!< Height the framebuffer was sized for.       */
  uint32_t BytesPerPixel; /*!< 2 or 4, derived from PixelFormat.           */
  uint8_t  Configured;    /*!< 1 once BSP_LCD_ConfigLayer() has succeeded. */
  uint8_t  Visible;       /*!< 0 while BSP_LCD_SetLayerVisible() hides it. */
} LCD_LayerCtx_t;

static LTDC_HandleTypeDef hlcd_ltdc;
static LCD_LayerCtx_t     LcdLayerCtx[BSP_LCD_LAYERS_NBR];
static uint32_t           LcdActiveLayer;
static uint32_t           LcdClockDivider;
static uint8_t            LcdInitialized;

static const struct
{
  GPIO_TypeDef *Port;
  uint32_t      Pins;
  uint32_t      Alternate;
} LcdPins[] =
{
  /* PD8 R7, PD13 R6 */
  { GPIOD, GPIO_PIN_8 | GPIO_PIN_13, GPIO_AF14_LCD },
  /* PF4 R3, PF7 VSYNC */
  { GPIOF, GPIO_PIN_4 | GPIO_PIN_7, GPIO_AF14_LCD },
  /* PA0 G3, PA1 G2, PA2 B7, PA5 CLK, PA6 HSYNC, PA8 B6, PA9 B5, PA10 B4,
   * PA11 B3, PA15 R5 */
  { GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6 |
           GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15,
    GPIO_AF14_LCD },
  /* PB10 G7, PB11 G6, PB12 G5 */
  { GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_AF14_LCD },
  /* PG10 G4, PG13 DE */
  { GPIOG, GPIO_PIN_10 | GPIO_PIN_13, GPIO_AF14_LCD },
  /* PA7 R4, and only this pin, sits on AF10. */
  { GPIOA, GPIO_PIN_7, GPIO_AF10_LCD },
};

static void     LCD_RIFConfig(void);
static void     LCD_GpioConfig(void);
static void     LCD_GpioDeConfig(void);
static int32_t  LCD_ComputeClockDivider(uint32_t Hz, uint32_t *Divider);
static int32_t  LCD_ApplyClockDivider(uint32_t Divider);
static uint8_t  LCD_IsPixelFormatSupported(uint32_t PixelFormat);
static uint32_t LCD_BytesPerPixel(uint32_t PixelFormat);
static int32_t  LCD_GetLayerCtx(uint32_t Instance, uint32_t LayerIndex, LCD_LayerCtx_t **Ctx);
static int32_t  LCD_GetActiveLayerCtx(uint32_t Instance, LCD_LayerCtx_t **Ctx);
static int32_t  LCD_CommitLayer(uint32_t LayerIndex);
static void     LCD_KeepHidden(uint32_t LayerIndex, const LCD_LayerCtx_t *Ctx);
static uint32_t LCD_PixelAddress(const LCD_LayerCtx_t *Ctx, uint32_t Xpos, uint32_t Ypos);
static void     LCD_StorePixel(uint32_t Addr, uint32_t BytesPerPixel, uint32_t Color);
static uint32_t LCD_LoadPixel(uint32_t Addr, uint32_t BytesPerPixel);

/* Grant both LTDC read masters (one per layer; with one grant the second layer stays
 * blank) and its slave ports. Compiled for the secure state only. */
static void LCD_RIFConfig(void)
{
#if defined(CPU_IN_SECURE_STATE)
  RIMC_MasterConfig_t master = {0};

  __HAL_RCC_RIFSC_CLK_ENABLE();

  master.MasterCID = RIF_CID_1;
  master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1, &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2, &master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
#endif /* CPU_IN_SECURE_STATE */
}

/* RGB565 wiring: R3-R7, G2-G7, B3-B7, CLK, DE, HSYNC, VSYNC. PA15 (R5) is JTDI: JTAG
 * is lost, SWD is unaffected. */
static void LCD_GpioConfig(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t         i;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  for (i = 0U; i < (sizeof(LcdPins) / sizeof(LcdPins[0])); i++)
  {
    gpio.Pin       = LcdPins[i].Pins;
    gpio.Alternate = LcdPins[i].Alternate;
    HAL_GPIO_Init(LcdPins[i].Port, &gpio);
  }
}

/* Pins back to analogue, the reset and lowest-power mode. */
static void LCD_GpioDeConfig(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t         i;

  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;

  for (i = 0U; i < (sizeof(LcdPins) / sizeof(LcdPins[0])); i++)
  {
    gpio.Pin = LcdPins[i].Pins;
    HAL_GPIO_Init(LcdPins[i].Port, &gpio);
  }
}

/* IC16 divider for Hz, from the PLL4 frequency read back. */
static int32_t LCD_ComputeClockDivider(uint32_t Hz, uint32_t *Divider)
{
  uint32_t pll4;
  uint32_t denominator;
  uint32_t divider;

  pll4 = HAL_RCCEx_GetPLL4CLKFreq();
  if (pll4 == 0U)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  /* Refresh = PLL4 / (divider * clocks per frame), rounded to nearest. */
  denominator = Hz * LCD_PIXELS_PER_FRAME;
  divider     = (pll4 + (denominator / 2U)) / denominator;

  if ((divider < LCD_CLK_DIVIDER_MIN) || (divider > LCD_CLK_DIVIDER_MAX))
  {
    /* PLL4 cannot reach this rate with an integer divider. */
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  *Divider = divider;
  return BSP_ERROR_NONE;
}

/* LTDC kernel clock: PLL4 through IC16, divided by 1 to 256. */
static int32_t LCD_ApplyClockDivider(uint32_t Divider)
{
  RCC_PeriphCLKInitTypeDef periph_clk = {0};

  periph_clk.PeriphClockSelection                    = RCC_PERIPHCLK_LTDC;
  periph_clk.LtdcClockSelection                      = RCC_LTDCCLKSOURCE_IC16;
  periph_clk.ICSelection[RCC_IC16].ClockSelection    = RCC_ICCLKSOURCE_PLL4;
  periph_clk.ICSelection[RCC_IC16].ClockDivider      = Divider;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static uint8_t LCD_IsPixelFormatSupported(uint32_t PixelFormat)
{
  uint8_t supported = 0U;

  if ((PixelFormat == BSP_LCD_PIXEL_FORMAT_RGB565) ||
      (PixelFormat == BSP_LCD_PIXEL_FORMAT_ARGB4444) ||
      (PixelFormat == BSP_LCD_PIXEL_FORMAT_ARGB8888))
  {
    supported = 1U;
  }

  return supported;
}

static uint32_t LCD_BytesPerPixel(uint32_t PixelFormat)
{
  return (PixelFormat == BSP_LCD_PIXEL_FORMAT_ARGB8888) ? 4U : 2U;
}

static int32_t LCD_GetLayerCtx(uint32_t Instance, uint32_t LayerIndex, LCD_LayerCtx_t **Ctx)
{
  if ((Instance >= BSP_LCD_INSTANCES_NBR) || (LayerIndex >= BSP_LCD_LAYERS_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *Ctx = &LcdLayerCtx[LayerIndex];
  return BSP_ERROR_NONE;
}

/* The drawing primitives' layer; BSP_ERROR_NO_INIT until it has a framebuffer. */
static int32_t LCD_GetActiveLayerCtx(uint32_t Instance, LCD_LayerCtx_t **Ctx)
{
  int32_t status;

  status = LCD_GetLayerCtx(Instance, LcdActiveLayer, Ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((*Ctx)->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return BSP_ERROR_NONE;
}

/* Reload through the layer's own register (the HAL masks LTDC_SRCR): at vertical
 * blanking while the LTDC runs, at once while it is off. */
static int32_t LCD_CommitLayer(uint32_t LayerIndex)
{
  uint32_t reload_type;

  if ((LTDC->GCR & LTDC_GCR_LTDCEN_Msk) != 0U)
  {
    reload_type = LTDC_RELOAD_VERTICAL_BLANKING;
  }
  else
  {
    reload_type = LTDC_RELOAD_IMMEDIATE;
  }

  if (HAL_LTDC_ReloadLayer(&hlcd_ltdc, reload_type, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* HAL layer setters set the enable bit; clear it again for a hidden layer. */
static void LCD_KeepHidden(uint32_t LayerIndex, const LCD_LayerCtx_t *Ctx)
{
  if (Ctx->Visible == 0U)
  {
    __HAL_LTDC_LAYER_DISABLE(&hlcd_ltdc, LayerIndex);
  }
}

static uint32_t LCD_PixelAddress(const LCD_LayerCtx_t *Ctx, uint32_t Xpos, uint32_t Ypos)
{
  return Ctx->Address + ((((Ypos * Ctx->Width) + Xpos)) * Ctx->BytesPerPixel);
}

static void LCD_StorePixel(uint32_t Addr, uint32_t BytesPerPixel, uint32_t Color)
{
  if (BytesPerPixel == 2U)
  {
    *(uint16_t *)Addr = (uint16_t)Color;
  }
  else
  {
    *(uint32_t *)Addr = Color;
  }
}

static uint32_t LCD_LoadPixel(uint32_t Addr, uint32_t BytesPerPixel)
{
  uint32_t color;

  if (BytesPerPixel == 2U)
  {
    color = (uint32_t)(*(const uint16_t *)Addr);
  }
  else
  {
    color = *(const uint32_t *)Addr;
  }

  return color;
}

/** @brief Set up the LTDC (Width, Height must be the panel's); returns with the display
  * off and both layers disabled, without framebuffers. */
int32_t BSP_LCD_Init(uint32_t Instance, uint32_t Width, uint32_t Height)
{
  int32_t  status;
  uint32_t divider = 0U;
  uint32_t layer;

  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((Width != BSP_LCD_WIDTH) || (Height != BSP_LCD_HEIGHT))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Grants first: the controller must never fetch before the RIF knows it. */
  LCD_RIFConfig();

  status = LCD_ComputeClockDivider(BSP_LCD_REFRESH_RATE_DEFAULT, &divider);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = LCD_ApplyClockDivider(divider);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }
  LcdClockDivider = divider;

  __HAL_RCC_LTDC_CLK_ENABLE();
  LCD_GpioConfig();

  hlcd_ltdc.Instance = LTDC;

  hlcd_ltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hlcd_ltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hlcd_ltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hlcd_ltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  /* The LTDC wants accumulated counts, each programmed as the count minus one. */
  hlcd_ltdc.Init.HorizontalSync     = LCD_HSYNC_WIDTH - 1U;
  hlcd_ltdc.Init.VerticalSync       = LCD_VSYNC_HEIGHT - 1U;
  hlcd_ltdc.Init.AccumulatedHBP     = (LCD_HSYNC_WIDTH + LCD_HBP) - 1U;
  hlcd_ltdc.Init.AccumulatedVBP     = (LCD_VSYNC_HEIGHT + LCD_VBP) - 1U;
  hlcd_ltdc.Init.AccumulatedActiveW = (LCD_HSYNC_WIDTH + LCD_HBP + BSP_LCD_WIDTH) - 1U;
  hlcd_ltdc.Init.AccumulatedActiveH = (LCD_VSYNC_HEIGHT + LCD_VBP + BSP_LCD_HEIGHT) - 1U;
  hlcd_ltdc.Init.TotalWidth         = LCD_HTOTAL - 1U;
  hlcd_ltdc.Init.TotalHeigh         = LCD_VTOTAL - 1U;

  /* Black behind everything, so an undersized layer letterboxes cleanly. */
  hlcd_ltdc.Init.Backcolor.Red   = 0U;
  hlcd_ltdc.Init.Backcolor.Green = 0U;
  hlcd_ltdc.Init.Backcolor.Blue  = 0U;

  /* Reset state, so a re-init takes the same path as the first. */
  hlcd_ltdc.State = HAL_LTDC_STATE_RESET;

  if (HAL_LTDC_Init(&hlcd_ltdc) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* HAL_LTDC_Init() leaves the controller enabled. */
  __HAL_LTDC_LAYER_DISABLE(&hlcd_ltdc, BSP_LCD_LAYER_0);
  __HAL_LTDC_LAYER_DISABLE(&hlcd_ltdc, BSP_LCD_LAYER_1);
  (void)HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_IMMEDIATE, BSP_LCD_LAYER_0);
  (void)HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_IMMEDIATE, BSP_LCD_LAYER_1);
  __HAL_LTDC_DISABLE(&hlcd_ltdc);

  for (layer = 0U; layer < BSP_LCD_LAYERS_NBR; layer++)
  {
    LcdLayerCtx[layer].Address       = 0U;
    LcdLayerCtx[layer].PixelFormat   = 0U;
    LcdLayerCtx[layer].Width         = 0U;
    LcdLayerCtx[layer].Height        = 0U;
    LcdLayerCtx[layer].BytesPerPixel = 0U;
    LcdLayerCtx[layer].Configured    = 0U;
  }
  LcdActiveLayer = BSP_LCD_LAYER_0;

  /* Clear bring-up errors: the first BSP_LCD_GetStatus() reports the application's. */
  LTDC->ICR = LCD_ICR_ERROR_FLAGS;

  LcdInitialized = 1U;
  return BSP_ERROR_NONE;
}

/** @brief Release the LTDC and its pins; the panel may briefly keep the last image. */
int32_t BSP_LCD_DeInit(uint32_t Instance)
{
  uint32_t layer;

  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (HAL_LTDC_DeInit(&hlcd_ltdc) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Read back before gating the clock, or a posted write left pending stalls APB5. */
  (void)READ_REG(hlcd_ltdc.Instance->GCR);
  __DSB();
  __HAL_RCC_LTDC_CLK_DISABLE();
  LCD_GpioDeConfig();

  for (layer = 0U; layer < BSP_LCD_LAYERS_NBR; layer++)
  {
    LcdLayerCtx[layer].Address       = 0U;
    LcdLayerCtx[layer].PixelFormat   = 0U;
    LcdLayerCtx[layer].Width         = 0U;
    LcdLayerCtx[layer].Height        = 0U;
    LcdLayerCtx[layer].BufWidth      = 0U;
    LcdLayerCtx[layer].BufHeight     = 0U;
    LcdLayerCtx[layer].BytesPerPixel = 0U;
    LcdLayerCtx[layer].Configured    = 0U;
    LcdLayerCtx[layer].Visible       = 0U;
  }

  LcdActiveLayer  = 0U;
  LcdClockDivider = 0U;
  LcdInitialized  = 0U;

  return BSP_ERROR_NONE;
}

/**
  * @brief  Give a layer its window, pixel format and framebuffer; Config is copied.
  * @note   Takes effect at once, even on a visible layer, and leaves the layer enabled.
  *         A 24-bit format returns BSP_ERROR_FEATURE_NOT_SUPPORTED.
  */
int32_t BSP_LCD_ConfigLayer(uint32_t Instance, uint32_t LayerIndex,
                            const BSP_LCD_LayerConfig_t *Config)
{
  LTDC_LayerCfgTypeDef layer_cfg = {0};
  LCD_LayerCtx_t      *ctx;
  int32_t              status;

  if (Config == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Config->X1 <= Config->X0) || (Config->Y1 <= Config->Y0) ||
      (Config->X1 > BSP_LCD_WIDTH) || (Config->Y1 > BSP_LCD_HEIGHT))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LCD_IsPixelFormatSupported(Config->PixelFormat) == 0U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if ((Config->Address == 0U) || ((Config->Address % BSP_LCD_FB_ALIGNMENT) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  layer_cfg.WindowX0      = Config->X0;
  layer_cfg.WindowX1      = Config->X1;
  layer_cfg.WindowY0      = Config->Y0;
  layer_cfg.WindowY1      = Config->Y1;
  layer_cfg.PixelFormat   = Config->PixelFormat;
  layer_cfg.FBStartAdress = Config->Address;
  layer_cfg.Alpha         = Config->Alpha;
  layer_cfg.Alpha0        = 0U;
  layer_cfg.ImageWidth    = Config->X1 - Config->X0;
  layer_cfg.ImageHeight   = Config->Y1 - Config->Y0;
  layer_cfg.Backcolor.Red   = 0U;
  layer_cfg.Backcolor.Green = 0U;
  layer_cfg.Backcolor.Blue  = 0U;

  if (Config->PixelFormat == BSP_LCD_PIXEL_FORMAT_RGB565)
  {
    /* No alpha channel to blend with, so constant alpha alone. */
    layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  }
  else
  {
    /* Pixel alpha times constant alpha: transparent pixels stay transparent. */
    layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  }

  if (HAL_LTDC_ConfigLayer(&hlcd_ltdc, &layer_cfg, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  ctx->Address       = Config->Address;
  ctx->PixelFormat   = Config->PixelFormat;
  ctx->Width         = Config->X1 - Config->X0;
  ctx->Height        = Config->Y1 - Config->Y0;
  /* The framebuffer's size: the ceiling for BSP_LCD_SetLayerWindow(). */
  ctx->BufWidth      = ctx->Width;
  ctx->BufHeight     = ctx->Height;
  ctx->BytesPerPixel = LCD_BytesPerPixel(Config->PixelFormat);
  ctx->Configured    = 1U;
  ctx->Visible       = 1U;

  return BSP_ERROR_NONE;
}

/** @brief Repoint a layer at once (tears while scanning). No cache maintenance: the
  * buffer must already be coherent. */
int32_t BSP_LCD_SetLayerAddress(uint32_t Instance, uint32_t LayerIndex, uint32_t Address)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((Address == 0U) || ((Address % BSP_LCD_FB_ALIGNMENT) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, Address, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  LCD_KeepHidden(LayerIndex, ctx);

  if (HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_IMMEDIATE, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  ctx->Address = Address;
  return BSP_ERROR_NONE;
}

/** @brief Buffer flip: clean the buffer from the D-cache (if USE_BSP_CPU_CACHE_MAINTENANCE),
  * repoint at the next vertical blank. Returns once queued; BSP_LCD_IsReloadPending()
  * reports the swap. */
int32_t BSP_LCD_SetLayerAddressVBlank(uint32_t Instance, uint32_t LayerIndex, uint32_t Address)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((Address == 0U) || ((Address % BSP_LCD_FB_ALIGNMENT) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  LCD_CACHE_CLEAN(Address, ctx->Width * ctx->Height * ctx->BytesPerPixel);

  if (HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, Address, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  LCD_KeepHidden(LayerIndex, ctx);

  status = LCD_CommitLayer(LayerIndex);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  ctx->Address = Address;
  return BSP_ERROR_NONE;
}

/** @brief Pending = 1 while a vertical-blank reload is queued, 0 once taken (the
  * previous buffer is then free to redraw). */
int32_t BSP_LCD_IsReloadPending(uint32_t Instance, uint32_t LayerIndex, uint32_t *Pending)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  if (Pending == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  *Pending = ((LTDC_LAYER(&hlcd_ltdc, LayerIndex)->RCR & LTDC_LxRCR_VBR_Msk) != 0U) ? 1U : 0U;
  return BSP_ERROR_NONE;
}

/** @brief Show or hide a layer; BSP_ERROR_NO_INIT to show one with no framebuffer. */
int32_t BSP_LCD_SetLayerVisible(uint32_t Instance, uint32_t LayerIndex, FunctionalState State)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (State == ENABLE)
  {
    if (ctx->Configured == 0U)
    {
      return BSP_ERROR_NO_INIT;
    }
    __HAL_LTDC_LAYER_ENABLE(&hlcd_ltdc, LayerIndex);
    ctx->Visible = 1U;
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&hlcd_ltdc, LayerIndex);
    ctx->Visible = 0U;
  }

  return LCD_CommitLayer(LayerIndex);
}

/** @brief Constant alpha, 0 transparent to 255 opaque; it scales ARGB pixels' own alpha. */
int32_t BSP_LCD_SetTransparency(uint32_t Instance, uint32_t LayerIndex, uint8_t Transparency)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* BSP_LCD_ConfigLayer() would overwrite it. */
  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (HAL_LTDC_SetAlpha_NoReload(&hlcd_ltdc, (uint32_t)Transparency, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return LCD_CommitLayer(LayerIndex);
}

int32_t BSP_LCD_SetLayerWindow(uint32_t Instance, uint32_t LayerIndex,
                               uint32_t Xpos, uint32_t Ypos,
                               uint32_t Width, uint32_t Height)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((Width == 0U) || (Height == 0U) ||
      (Xpos >= BSP_LCD_WIDTH) || (Ypos >= BSP_LCD_HEIGHT) ||
      (Width > (BSP_LCD_WIDTH - Xpos)) || (Height > (BSP_LCD_HEIGHT - Ypos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Against the framebuffer size, so a shrunk window can grow back. */
  if ((Width > ctx->BufWidth) || (Height > ctx->BufHeight))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Size and position staged together, committed once. */
  if (HAL_LTDC_SetWindowSize_NoReload(&hlcd_ltdc, Width, Height, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_LTDC_SetWindowPosition_NoReload(&hlcd_ltdc, Xpos, Ypos, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  LCD_KeepHidden(LayerIndex, ctx);

  status = LCD_CommitLayer(LayerIndex);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  ctx->Width  = Width;
  ctx->Height = Height;

  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_SetColorKeying(uint32_t Instance, uint32_t LayerIndex, uint32_t Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Staged by hand: the HAL's keying calls reload at once, which would pull a flip
   * queued for the vertical blank into mid-frame. */
  WRITE_REG(LTDC_LAYER(&hlcd_ltdc, LayerIndex)->CKCR, Color & 0x00FFFFFFU);

  if (HAL_LTDC_EnableColorKeying_NoReload(&hlcd_ltdc, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return LCD_CommitLayer(LayerIndex);
}

int32_t BSP_LCD_ResetColorKeying(uint32_t Instance, uint32_t LayerIndex)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (ctx->Configured == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (HAL_LTDC_DisableColorKeying_NoReload(&hlcd_ltdc, LayerIndex) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return LCD_CommitLayer(LayerIndex);
}

/* Per-layer reload registers, back to back: the HAL masks both layers from LTDC_SRCR. */
int32_t BSP_LCD_Reload(uint32_t Instance, uint32_t Immediate)
{
  uint32_t reload_type;
  uint32_t layer;

  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* A vertical-blank reload never completes while the LTDC is stopped. */
  if ((Immediate != 0U) || ((LTDC->GCR & LTDC_GCR_LTDCEN_Msk) == 0U))
  {
    reload_type = LTDC_RELOAD_IMMEDIATE;
  }
  else
  {
    reload_type = LTDC_RELOAD_VERTICAL_BLANKING;
  }

  for (layer = 0U; layer < BSP_LCD_LAYERS_NBR; layer++)
  {
    if (HAL_LTDC_ReloadLayer(&hlcd_ltdc, reload_type, layer) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return BSP_ERROR_NONE;
}

/** @brief Choose the layer the drawing primitives write into. */
int32_t BSP_LCD_SetActiveLayer(uint32_t Instance, uint32_t LayerIndex)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  status = LCD_GetLayerCtx(Instance, LayerIndex, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  LcdActiveLayer = LayerIndex;
  return BSP_ERROR_NONE;
}

/** @brief Pixel format of the layer the drawing primitives write into. */
int32_t BSP_LCD_GetPixelFormat(uint32_t Instance, uint32_t *PixelFormat)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;

  if (PixelFormat == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  *PixelFormat = ctx->PixelFormat;
  return BSP_ERROR_NONE;
}

/** @brief Enable the LTDC; the board has no backlight, panel power or reset control. */
int32_t BSP_LCD_DisplayOn(uint32_t Instance)
{
  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  __HAL_LTDC_ENABLE(&hlcd_ltdc);
  return BSP_ERROR_NONE;
}

/** @brief Disable the LTDC; the syncs and pixel clock stop with it. */
int32_t BSP_LCD_DisplayOff(uint32_t Instance)
{
  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  __HAL_LTDC_DISABLE(&hlcd_ltdc);
  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *XSize)
{
  if ((Instance >= BSP_LCD_INSTANCES_NBR) || (XSize == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *XSize = BSP_LCD_WIDTH;
  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *YSize)
{
  if ((Instance >= BSP_LCD_INSTANCES_NBR) || (YSize == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *YSize = BSP_LCD_HEIGHT;
  return BSP_ERROR_NONE;
}

/**
  * @brief  Set the refresh rate in Hz, BSP_LCD_REFRESH_RATE_MIN..MAX; others are refused.
  * @note   PLL4 is read back, not assumed: boot loaders set it differently.
  * @note   Lower the rate if BSP_LCD_GetStatus() reports underruns. A change while
  *         scanning can disturb a frame or two.
  */
int32_t BSP_LCD_SetRefreshRate(uint32_t Instance, uint32_t Hz)
{
  int32_t  status;
  uint32_t divider = 0U;

  if (Instance >= BSP_LCD_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((Hz < BSP_LCD_REFRESH_RATE_MIN) || (Hz > BSP_LCD_REFRESH_RATE_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_ComputeClockDivider(Hz, &divider);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = LCD_ApplyClockDivider(divider);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  LcdClockDivider = divider;
  return BSP_ERROR_NONE;
}

/** @brief Achieved refresh rate in Hz (rounded), from PLL4 and the divider in force. */
int32_t BSP_LCD_GetRefreshRate(uint32_t Instance, uint32_t *Hz)
{
  uint32_t pll4;
  uint32_t clocks_per_frame;

  if ((Instance >= BSP_LCD_INSTANCES_NBR) || (Hz == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  pll4 = HAL_RCCEx_GetPLL4CLKFreq();
  if (pll4 == 0U)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  clocks_per_frame = LcdClockDivider * LCD_PIXELS_PER_FRAME;
  *Hz = (pll4 + (clocks_per_frame / 2U)) / clocks_per_frame;

  return BSP_ERROR_NONE;
}

/**
  * @brief  Read and clear the error flags, so each covers the time since the last call.
  * @note   An underrun means AXI reads could not keep up: torn or shifted lines.
  *         The line and reload interrupt flags are left to the application.
  */
int32_t BSP_LCD_GetStatus(uint32_t Instance, BSP_LCD_Status_t *Status)
{
  uint32_t isr;

  if ((Instance >= BSP_LCD_INSTANCES_NBR) || (Status == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (LcdInitialized == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  isr = LTDC->ISR;

  Status->Underrun         = ((isr & LTDC_ISR_FUIF_Msk) != 0U) ? 1U : 0U;
  Status->UnderrunWarning  = ((isr & LTDC_ISR_FUWIF_Msk) != 0U) ? 1U : 0U;
  Status->UnderrunRotation = ((isr & LTDC_ISR_FURIF_Msk) != 0U) ? 1U : 0U;
  Status->TransferError    = ((isr & LTDC_ISR_TERRIF_Msk) != 0U) ? 1U : 0U;
  Status->Enabled          = ((LTDC->GCR & LTDC_GCR_LTDCEN_Msk) != 0U) ? 1U : 0U;
  Status->CurrentLine      = LTDC->CPSR & LTDC_CPSR_CYPOS_Msk;

  LTDC->ICR = LCD_ICR_ERROR_FLAGS;

  return BSP_ERROR_NONE;
}

/* Drawing primitives: CPU only, in the active layer's coordinates and format;
 * BSP_ERROR_NO_INIT until it has a framebuffer. 16-bit formats use Color's low half. */

int32_t BSP_LCD_WritePixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        addr;

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Xpos >= ctx->Width) || (Ypos >= ctx->Height))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  addr = LCD_PixelAddress(ctx, Xpos, Ypos);
  LCD_StorePixel(addr, ctx->BytesPerPixel, Color);
  LCD_CACHE_CLEAN(addr, ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

/** @brief Read one pixel, cleaning and invalidating its cache line first. */
int32_t BSP_LCD_ReadPixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t *Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        addr;

  if (Color == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Xpos >= ctx->Width) || (Ypos >= ctx->Height))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  addr = LCD_PixelAddress(ctx, Xpos, Ypos);
  LCD_CACHE_CLEAN_INVALIDATE(addr, ctx->BytesPerPixel);
  *Color = LCD_LoadPixel(addr, ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_FillRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                         uint32_t Width, uint32_t Height, uint32_t Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        row;
  uint32_t        col;
  uint32_t        row_addr;
  uint32_t        stride;

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Width == 0U) || (Height == 0U) ||
      (Xpos >= ctx->Width) || (Ypos >= ctx->Height) ||
      (Width > (ctx->Width - Xpos)) || (Height > (ctx->Height - Ypos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  stride   = ctx->Width * ctx->BytesPerPixel;
  row_addr = LCD_PixelAddress(ctx, Xpos, Ypos);

  for (row = 0U; row < Height; row++)
  {
    uint32_t addr = row_addr;

    for (col = 0U; col < Width; col++)
    {
      LCD_StorePixel(addr, ctx->BytesPerPixel, Color);
      addr += ctx->BytesPerPixel;
    }

    row_addr += stride;
  }

  /* Clean the bounding box in one pass; the extra is harmless, a clean discards nothing. */
  LCD_CACHE_CLEAN(LCD_PixelAddress(ctx, Xpos, Ypos),
                  (((Height - 1U) * ctx->Width) + Width) * ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

/** @brief Copy a block into the active layer: pData packed in the layer's format
  * (no conversion), little-endian, no row padding, any alignment. */
int32_t BSP_LCD_FillRGBRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                            const uint8_t *pData, uint32_t Width, uint32_t Height)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        row;
  uint32_t        col;
  uint32_t        row_addr;
  uint32_t        stride;
  const uint8_t  *src;

  if (pData == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Width == 0U) || (Height == 0U) ||
      (Xpos >= ctx->Width) || (Ypos >= ctx->Height) ||
      (Width > (ctx->Width - Xpos)) || (Height > (ctx->Height - Ypos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  stride   = ctx->Width * ctx->BytesPerPixel;
  row_addr = LCD_PixelAddress(ctx, Xpos, Ypos);
  src      = pData;

  for (row = 0U; row < Height; row++)
  {
    uint32_t addr = row_addr;

    for (col = 0U; col < Width; col++)
    {
      /* Byte by byte: pData need not be aligned. */
      uint32_t color = (uint32_t)src[0] | ((uint32_t)src[1] << 8);

      if (ctx->BytesPerPixel == 4U)
      {
        color |= ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
      }

      LCD_StorePixel(addr, ctx->BytesPerPixel, color);
      addr += ctx->BytesPerPixel;
      src  += ctx->BytesPerPixel;
    }

    row_addr += stride;
  }

  LCD_CACHE_CLEAN(LCD_PixelAddress(ctx, Xpos, Ypos),
                  (((Height - 1U) * ctx->Width) + Width) * ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

/* Little-endian header field of Size (2 or 4) bytes. */
static uint32_t LCD_BmpField(const uint8_t *pHdr, uint32_t Offset, uint32_t Size)
{
  uint32_t value = 0U;
  uint32_t i;

  for (i = 0U; i < Size; i++)
  {
    value |= ((uint32_t)pHdr[Offset + i]) << (8U * i);
  }

  return value;
}

/* Channel positions in a bitmap pixel. */
typedef struct
{
  uint32_t Mask[4];   /*!< Red, green, blue and alpha; no alpha mask means opaque */
  uint32_t Shift[4];  /*!< Where each channel's top eight bits, or fewer, start    */
  uint32_t Max[4];    /*!< Each channel's full scale after the shift, at most 255 */
} LCD_BmpLayout_t;

/* BI_BITFIELDS (3): masks after the 40-byte header, alpha too in a longer header.
 * BI_RGB (0): 16 bits is 5-5-5, 32 bits has an unused fourth byte (not alpha).
 * A channel wider than 8 bits keeps its top 8. */
static void LCD_BmpLayout(LCD_BmpLayout_t *Layout, const uint8_t *pBmp, uint32_t Bpp,
                          uint32_t Compression)
{
  uint32_t i;

  if (Compression == 3U)
  {
    Layout->Mask[0] = LCD_BmpField(pBmp, 54U, 4U);
    Layout->Mask[1] = LCD_BmpField(pBmp, 58U, 4U);
    Layout->Mask[2] = LCD_BmpField(pBmp, 62U, 4U);
    Layout->Mask[3] = (LCD_BmpField(pBmp, 14U, 4U) >= 56U) ? LCD_BmpField(pBmp, 66U, 4U) : 0U;
  }
  else if (Bpp == 16U)
  {
    Layout->Mask[0] = 0x7C00U;
    Layout->Mask[1] = 0x03E0U;
    Layout->Mask[2] = 0x001FU;
    Layout->Mask[3] = 0U;
  }
  else
  {
    Layout->Mask[0] = 0x00FF0000U;
    Layout->Mask[1] = 0x0000FF00U;
    Layout->Mask[2] = 0x000000FFU;
    Layout->Mask[3] = 0U;
  }

  for (i = 0U; i < 4U; i++)
  {
    uint32_t shift = 0U;
    uint32_t max   = 0U;

    if (Layout->Mask[i] != 0U)
    {
      while (((Layout->Mask[i] >> shift) & 1U) == 0U)
      {
        shift++;
      }
      max = Layout->Mask[i] >> shift;
      while (max > 0xFFU)
      {
        max >>= 1;
        shift++;
      }
    }

    Layout->Shift[i] = shift;
    Layout->Max[i]   = max;
  }
}

/* Channel 0-3 (R, G, B, A) scaled to 0-255, rounded; Absent if the bitmap has none. */
static uint32_t LCD_BmpChannel(uint32_t Pixel, const LCD_BmpLayout_t *Layout, uint32_t Channel,
                               uint32_t Absent)
{
  const uint32_t max = Layout->Max[Channel];
  uint32_t       value;

  if (max == 0U)
  {
    return Absent;
  }

  value = (Pixel >> Layout->Shift[Channel]) & max;

  return ((value * 255U) + (max / 2U)) / max;
}

/* One source pixel (SrcBytes 2, 3 or 4) in the layer's format. */
static uint32_t LCD_BmpPixel(const uint8_t *pSrc, uint32_t SrcBytes, const LCD_BmpLayout_t *Layout,
                             uint32_t PixelFormat)
{
  uint32_t pixel = 0U;
  uint32_t i;
  uint32_t r;
  uint32_t g;
  uint32_t b;
  uint32_t a;

  for (i = 0U; i < SrcBytes; i++)
  {
    pixel |= ((uint32_t)pSrc[i]) << (8U * i);
  }

  r = LCD_BmpChannel(pixel, Layout, 0U, 0U);
  g = LCD_BmpChannel(pixel, Layout, 1U, 0U);
  b = LCD_BmpChannel(pixel, Layout, 2U, 0U);
  a = LCD_BmpChannel(pixel, Layout, 3U, 0xFFU);

  if (PixelFormat == BSP_LCD_PIXEL_FORMAT_RGB565)
  {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
  }
  if (PixelFormat == BSP_LCD_PIXEL_FORMAT_ARGB4444)
  {
    return ((a >> 4) << 12) | ((r >> 4) << 8) | ((g >> 4) << 4) | (b >> 4);
  }

  return (a << 24) | (r << 16) | (g << 8) | b;
}

int32_t BSP_LCD_DrawBitmap(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, const uint8_t *pBmp)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        data_offset;
  uint32_t        width;
  int32_t         signed_height;
  uint32_t        height;
  uint32_t        bpp;
  uint32_t        compression;
  uint32_t        src_bytes;
  uint32_t        src_stride;
  uint32_t        row;
  uint32_t        col;
  uint32_t        row_addr;
  uint32_t        dst_stride;
  uint32_t        top_down;
  const uint8_t  *data;
  LCD_BmpLayout_t layout;

  if (pBmp == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* Check the signature before reading any field. */
  if ((pBmp[0] != (uint8_t)'B') || (pBmp[1] != (uint8_t)'M'))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  data_offset   = LCD_BmpField(pBmp, 10U, 4U);
  width         = LCD_BmpField(pBmp, 18U, 4U);
  signed_height = (int32_t)LCD_BmpField(pBmp, 22U, 4U);
  bpp           = LCD_BmpField(pBmp, 28U, 2U);
  compression   = LCD_BmpField(pBmp, 30U, 4U);

  /* Negative height: rows stored top to bottom. */
  if (signed_height < 0)
  {
    top_down = 1U;
    height   = (uint32_t)(-signed_height);
  }
  else
  {
    top_down = 0U;
    height   = (uint32_t)signed_height;
  }

  if ((bpp != 16U) && (bpp != 24U) && (bpp != 32U))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* 0 is BI_RGB, 3 is BI_BITFIELDS. Everything else is compressed. */
  if ((compression != 0U) && (compression != 3U))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if ((width == 0U) || (height == 0U) ||
      (Xpos >= ctx->Width) || (Ypos >= ctx->Height) ||
      (width > (ctx->Width - Xpos)) || (height > (ctx->Height - Ypos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  LCD_BmpLayout(&layout, pBmp, bpp, compression);

  src_bytes  = bpp / 8U;
  /* Rows are padded to four bytes. */
  src_stride = ((width * src_bytes) + 3U) & ~3U;
  dst_stride = ctx->Width * ctx->BytesPerPixel;
  row_addr   = LCD_PixelAddress(ctx, Xpos, Ypos);
  data       = &pBmp[data_offset];

  for (row = 0U; row < height; row++)
  {
    const uint8_t *src = (top_down != 0U)
                         ? &data[row * src_stride]
                         : &data[((height - 1U) - row) * src_stride];
    uint32_t addr = row_addr;

    for (col = 0U; col < width; col++)
    {
      LCD_StorePixel(addr, ctx->BytesPerPixel,
                     LCD_BmpPixel(src, src_bytes, &layout, ctx->PixelFormat));
      addr += ctx->BytesPerPixel;
      src  += src_bytes;
    }

    row_addr += dst_stride;
  }

  LCD_CACHE_CLEAN(LCD_PixelAddress(ctx, Xpos, Ypos),
                  (((height - 1U) * ctx->Width) + width) * ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_DrawHLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                          uint32_t Length, uint32_t Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        addr;
  uint32_t        start;
  uint32_t        col;

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Length == 0U) || (Xpos >= ctx->Width) || (Ypos >= ctx->Height) ||
      (Length > (ctx->Width - Xpos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  start = LCD_PixelAddress(ctx, Xpos, Ypos);
  addr  = start;

  for (col = 0U; col < Length; col++)
  {
    LCD_StorePixel(addr, ctx->BytesPerPixel, Color);
    addr += ctx->BytesPerPixel;
  }

  LCD_CACHE_CLEAN(start, Length * ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}

int32_t BSP_LCD_DrawVLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos,
                          uint32_t Length, uint32_t Color)
{
  LCD_LayerCtx_t *ctx;
  int32_t         status;
  uint32_t        addr;
  uint32_t        start;
  uint32_t        stride;
  uint32_t        row;

  status = LCD_GetActiveLayerCtx(Instance, &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Length == 0U) || (Xpos >= ctx->Width) || (Ypos >= ctx->Height) ||
      (Length > (ctx->Height - Ypos)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  stride = ctx->Width * ctx->BytesPerPixel;
  start  = LCD_PixelAddress(ctx, Xpos, Ypos);
  addr   = start;

  for (row = 0U; row < Length; row++)
  {
    LCD_StorePixel(addr, ctx->BytesPerPixel, Color);
    addr += stride;
  }

  /* One clean over the bounding box beats Length separate ones. */
  LCD_CACHE_CLEAN(start, (((Length - 1U) * ctx->Width) + 1U) * ctx->BytesPerPixel);

  return BSP_ERROR_NONE;
}
