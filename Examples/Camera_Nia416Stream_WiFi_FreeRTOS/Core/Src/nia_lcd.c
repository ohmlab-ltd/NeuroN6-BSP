/*
  nia_lcd.c

  The panel, when NIA_LCD is 1. Layer 0 is the camera: the dump pipe writes
  RGB565 frames straight into the buffer the display controller scans.
  Layer 1 is the ARGB4444 overlay of the detector's boxes, double-buffered
  and swapped at a vertical blank. Labels use ST's 16-pixel font (fonts.h,
  font16.c).

  The camera buffer is a single 768 KB picture in uncached AXISRAM, so fast
  movement can tear. The overlay buffers are in external PSRAM, which this
  example's MPU leaves uncached: no cache maintenance is needed.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: MIT
  This software is licensed under the terms in
  Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
*/

#include "nia_lcd.h"

#include "neuron6_lcd.h"
#include "fonts.h"
#include "LCD_Overlay_Nia.h"
#include "nia_pp.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#if NIA_LCD

#define PANEL_W  BSP_LCD_WIDTH
#define PANEL_H  BSP_LCD_HEIGHT

/* The camera's buffer, on chip, and the overlay's two. */
static uint8_t  CamBuf[PANEL_W * PANEL_H * 2U]
  __attribute__((section(".axisram_nc"), aligned(32)));
static uint16_t OvlBuf[2][PANEL_W * PANEL_H]
  __attribute__((section(".ext_psram"), aligned(32)));

static uint16_t          *OvlFront;   /* shown by the panel */
static uint16_t          *OvlBack;    /* drawn by the processor */
static volatile uint32_t  OvlDraws;
static volatile uint32_t  OvlStalls;

/* What each overlay buffer has drawn, so only that is cleared; a full list
 * clears the whole buffer. Text lies inside its background rectangle. */
typedef struct
{
  int16_t X0;
  int16_t Y0;
  int16_t X1;
  int16_t Y1;
} ovl_rect_t;

#define OVL_DIRTY_MAX  512U
static ovl_rect_t OvlDirty[2][OVL_DIRTY_MAX];
static uint32_t   OvlDirtyCount[2];
static uint8_t    OvlDirtyAll[2];
static uint32_t   OvlDrawing;         /* the buffer being drawn, 0 or 1 */

/* Where the camera's picture sits on the panel, in panel pixels. */
static int32_t CamX;
static int32_t CamY;
static int32_t CamW;
static int32_t CamH;

/* From the application: inference start and end (ms), the camera frame, the
 * window of it the network sees, and the class names. */
extern uint32_t ts[];
extern uint16_t g_nn_crop_x;
extern uint16_t g_nn_crop_y;
extern uint16_t g_nn_crop_w;
extern uint16_t g_nn_crop_h;
extern uint16_t g_camera_full_w;
extern uint16_t g_camera_full_h;

#ifndef NB_CLASSES
#define NB_CLASSES              80
#endif
extern const char *classes_table[NB_CLASSES];

/* Appearance */

#define ARGB4444(a, r, g, b) \
  ((uint16_t)((((a) & 0xFU) << 12) | (((r) & 0xFU) << 8) | \
              (((g) & 0xFU) << 4) | ((b) & 0xFU)))

#define OVL_WHITE        ARGB4444(0xFU, 0xFU, 0xFU, 0xFU)
#define OVL_LABEL_BG     ARGB4444(0xAU, 0x0U, 0x0U, 0x0U)   /* translucent black */
#define OVL_HUD_BG       ARGB4444(0x8U, 0x0U, 0x0U, 0x0U)   /* translucent black */
#define OVL_LABEL_PAD    3
#define OVL_HUD_PAD_X    4
#define OVL_HUD_PAD_Y    3
#define OVL_BOX_THICK    2

/* One opaque colour a class, eight in turn. */
static const uint16_t ClassColours[8] =
{
  ARGB4444(0xFU, 0xFU, 0x0U, 0x0U),   /* red     */
  ARGB4444(0xFU, 0x0U, 0xFU, 0x0U),   /* green   */
  ARGB4444(0xFU, 0x2U, 0x8U, 0xFU),   /* blue    */
  ARGB4444(0xFU, 0xFU, 0xFU, 0x0U),   /* yellow  */
  ARGB4444(0xFU, 0xFU, 0x0U, 0xFU),   /* magenta */
  ARGB4444(0xFU, 0x0U, 0xFU, 0xFU),   /* cyan    */
  ARGB4444(0xFU, 0xFU, 0x8U, 0x0U),   /* orange  */
  ARGB4444(0xFU, 0xBU, 0x6U, 0xFU),   /* violet  */
};

/* Drawing */

static void fill(uint16_t *pBuf, int32_t X0, int32_t Y0, int32_t X1, int32_t Y1,
                 uint16_t Colour)
{
  int32_t x;
  int32_t y;

  for (y = Y0; y < Y1; y++)
  {
    uint16_t *row = pBuf + ((uint32_t)y * PANEL_W);

    for (x = X0; x < X1; x++)
    {
      row[x] = Colour;
    }
  }
}

/* A filled rectangle, clipped to the panel and remembered for clearing;
 * X1 and Y1 are exclusive. */
static void fill_rect(uint16_t *pBuf, int32_t X0, int32_t Y0, int32_t X1, int32_t Y1,
                      uint16_t Colour)
{
  const uint32_t k = OvlDrawing;

  if (X0 < 0) { X0 = 0; }
  if (Y0 < 0) { Y0 = 0; }
  if (X1 > (int32_t)PANEL_W) { X1 = (int32_t)PANEL_W; }
  if (Y1 > (int32_t)PANEL_H) { Y1 = (int32_t)PANEL_H; }
  if ((X1 <= X0) || (Y1 <= Y0))
  {
    return;
  }
  fill(pBuf, X0, Y0, X1, Y1, Colour);

  if (OvlDirtyCount[k] < OVL_DIRTY_MAX)
  {
    ovl_rect_t *r = &OvlDirty[k][OvlDirtyCount[k]++];

    r->X0 = (int16_t)X0;
    r->Y0 = (int16_t)Y0;
    r->X1 = (int16_t)X1;
    r->Y1 = (int16_t)Y1;
  }
  else
  {
    OvlDirtyAll[k] = 1U;
  }
}

/* Clear what the last drawing in this buffer left. */
static void clear_drawn(uint16_t *pBuf, uint32_t K)
{
  uint32_t i;

  if (OvlDirtyAll[K] != 0U)
  {
    (void)memset(pBuf, 0, (size_t)PANEL_W * PANEL_H * sizeof(uint16_t));
  }
  else
  {
    for (i = 0U; i < OvlDirtyCount[K]; i++)
    {
      const ovl_rect_t *r = &OvlDirty[K][i];

      fill(pBuf, r->X0, r->Y0, r->X1, r->Y1, 0U);
    }
  }
  OvlDirtyCount[K] = 0U;
  OvlDirtyAll[K]   = 0U;
}

/* A hollow box OVL_BOX_THICK pixels wide; X1 and Y1 are exclusive. */
static void box_outline(uint16_t *pBuf, int32_t X0, int32_t Y0, int32_t X1, int32_t Y1,
                        uint16_t Colour)
{
  const int32_t t = OVL_BOX_THICK;

  if ((X1 <= X0) || (Y1 <= Y0))
  {
    return;
  }
  fill_rect(pBuf, X0, Y0, X1, Y0 + t, Colour);        /* top    */
  fill_rect(pBuf, X0, Y1 - t, X1, Y1, Colour);        /* bottom */
  fill_rect(pBuf, X0, Y0, X0 + t, Y1, Colour);        /* left   */
  fill_rect(pBuf, X1 - t, Y0, X1, Y1, Colour);        /* right  */
}

/* White text in ST's 16-pixel font (one bit a pixel, MSB first); the
 * background is left as it is. */
static void text(uint16_t *pBuf, int32_t X, int32_t Y, const char *pS)
{
  const uint32_t w        = Font16.Width;
  const uint32_t h        = Font16.Height;
  const uint32_t row_size = (w + 7U) / 8U;

  for (; *pS != '\0'; pS++, X += (int32_t)w)
  {
    const uint8_t c = (uint8_t)*pS;
    const uint8_t *glyph;
    uint32_t row;

    if ((c < 0x20U) || (c > 0x7EU))
    {
      continue;
    }
    glyph = &Font16.table[(uint32_t)(c - 0x20U) * h * row_size];
    for (row = 0U; row < h; row++)
    {
      const int32_t y = Y + (int32_t)row;
      uint32_t col;

      if ((y < 0) || (y >= (int32_t)PANEL_H))
      {
        continue;
      }
      for (col = 0U; col < w; col++)
      {
        const int32_t x = X + (int32_t)col;

        if ((x >= 0) && (x < (int32_t)PANEL_W) &&
            (((glyph[(row * row_size) + (col / 8U)] >> (7U - (col % 8U))) & 1U) != 0U))
        {
          pBuf[((uint32_t)y * PANEL_W) + (uint32_t)x] = OVL_WHITE;
        }
      }
    }
  }
}

/* A network coordinate (0 to 1 across its input) as a panel pixel, via the
 * crop window and where the picture sits on the panel. */
static int32_t nn_to_panel_x(float Xn)
{
  const float cam = ((float)g_nn_crop_x + (Xn * (float)g_nn_crop_w)) / (float)g_camera_full_w;

  return (int32_t)((float)CamX + (cam * (float)CamW));
}

static int32_t nn_to_panel_y(float Yn)
{
  const float cam = ((float)g_nn_crop_y + (Yn * (float)g_nn_crop_h)) / (float)g_camera_full_h;

  return (int32_t)((float)CamY + (cam * (float)CamH));
}

/* The panel */

uint8_t *NiaLcd_Init(uint32_t CamWidth, uint32_t CamHeight)
{
  BSP_LCD_LayerConfig_t layer = {0};

  if ((CamWidth == 0U) || (CamHeight == 0U) || (CamWidth > PANEL_W) || (CamHeight > PANEL_H))
  {
    return NULL;
  }
  if (BSP_LCD_Init(0, PANEL_W, PANEL_H) != BSP_ERROR_NONE)
  {
    return NULL;
  }

  CamX = (int32_t)((PANEL_W - CamWidth) / 2U);
  CamY = (int32_t)((PANEL_H - CamHeight) / 2U);
  CamW = (int32_t)CamWidth;
  CamH = (int32_t)CamHeight;

  /* Layer 0, the camera: centred, opaque, black until the first frame. */
  (void)memset(CamBuf, 0, sizeof(CamBuf));
  layer.X0          = (uint32_t)CamX;
  layer.X1          = (uint32_t)CamX + CamWidth;
  layer.Y0          = (uint32_t)CamY;
  layer.Y1          = (uint32_t)CamY + CamHeight;
  layer.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565;
  layer.Address     = (uint32_t)CamBuf;
  layer.Alpha       = 255U;
  if (BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_0, &layer) != BSP_ERROR_NONE)
  {
    return NULL;
  }

  /* Layer 1, the overlay: whole panel, per-pixel alpha, both buffers clear. */
  (void)memset(OvlBuf, 0, sizeof(OvlBuf));
  OvlDirtyCount[0] = 0U;
  OvlDirtyCount[1] = 0U;
  OvlFront = OvlBuf[0];
  OvlBack  = OvlBuf[1];

  layer.X0          = 0U;
  layer.X1          = PANEL_W;
  layer.Y0          = 0U;
  layer.Y1          = PANEL_H;
  layer.PixelFormat = BSP_LCD_PIXEL_FORMAT_ARGB4444;
  layer.Address     = (uint32_t)OvlFront;
  layer.Alpha       = 255U;
  if (BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_1, &layer) != BSP_ERROR_NONE)
  {
    OvlBack = NULL;
    return NULL;
  }

  if (BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE)
  {
    OvlBack = NULL;
    return NULL;
  }
  return CamBuf;
}

/* The buffer to draw into. The panel shows it until the last commit's swap at
 * a vertical blank, so wait: at most one frame, capped at 50 ms. */
uint16_t *OVL_GetBackBuffer(void)
{
  uint32_t pending = 0U;
  uint32_t start;
  uint8_t  stalled = 0U;

  if (OvlBack == NULL)
  {
    return NULL;
  }
  start = HAL_GetTick();
  while ((BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_1, &pending) == BSP_ERROR_NONE) &&
         (pending != 0U) && ((HAL_GetTick() - start) <= 50U))
  {
    stalled = 1U;
    vTaskDelay(1);
  }
  if (stalled != 0U)
  {
    OvlStalls++;
  }
  return OvlBack;
}

/* Show the buffer just drawn at the next vertical blank, and draw into the
 * other one next. */
void OVL_Commit(void)
{
  uint16_t *drawn = OvlBack;

  if (drawn == NULL)
  {
    return;
  }
  /* The drawing reaches the PSRAM before the panel is pointed at it. */
  __DSB();
  if (BSP_LCD_SetLayerAddressVBlank(0, BSP_LCD_LAYER_1, (uint32_t)drawn) != BSP_ERROR_NONE)
  {
    return;
  }
  OvlBack  = OvlFront;
  OvlFront = drawn;
  OvlDraws++;
}

uint32_t NiaLcd_Draws(void)
{
  return OvlDraws;
}

uint32_t NiaLcd_Stalls(void)
{
  return OvlStalls;
}

/* Called by the detector's post-processor after each inference: labelled
 * boxes and the inference time. Draws nothing without a panel. */
void LCD_Overlay_Nia_From(const struct nia_pp_out *out, OVL_Mode_t mode)
{
  uint16_t *buf = OVL_GetBackBuffer();
  int32_t   n;
  int32_t   d;

  if (buf == NULL)
  {
    return;
  }
  OvlDrawing = (buf == OvlBuf[0]) ? 0U : 1U;

  if ((mode == OVL_RELOAD_CLEAR) || (mode == OVL_CLEAR))
  {
    clear_drawn(buf, OvlDrawing);
  }

  /* The inference time, top left. */
  {
    char    s[32];
    int32_t bw;
    int32_t bh;
    const int32_t tx = 8;
    const int32_t ty = 8;

    (void)snprintf(s, sizeof(s), "inference: %lums", (unsigned long)(ts[1] - ts[0]));
    bw = ((int32_t)strlen(s) * (int32_t)Font16.Width) + (2 * OVL_HUD_PAD_X);
    bh = (int32_t)Font16.Height + (2 * OVL_HUD_PAD_Y);
    fill_rect(buf, tx, ty, tx + bw, ty + bh, OVL_HUD_BG);
    text(buf, tx + OVL_HUD_PAD_X, ty + OVL_HUD_PAD_Y, s);
  }

  /* Each detection: a box in its class's colour, "<class> <conf>%" above it. */
  n = ((out != NULL) && (out->pOutBuff != NULL)) ? out->nb_detect : 0;
  for (d = 0; d < n; d++)
  {
    const nia_det_t *det    = &out->pOutBuff[d];
    const uint16_t   colour = ClassColours[(uint32_t)det->class_index % 8U];
    const char      *name   = "?";
    char    label[48];
    int32_t bx0 = nn_to_panel_x(det->x_center - (det->width * 0.5f));
    int32_t by0 = nn_to_panel_y(det->y_center - (det->height * 0.5f));
    int32_t bx1 = nn_to_panel_x(det->x_center + (det->width * 0.5f));
    int32_t by1 = nn_to_panel_y(det->y_center + (det->height * 0.5f));
    int32_t lh;
    int32_t ly0;

    if (bx0 < 0) { bx0 = 0; }
    if (by0 < 0) { by0 = 0; }
    if (bx1 > (int32_t)PANEL_W) { bx1 = (int32_t)PANEL_W; }
    if (by1 > (int32_t)PANEL_H) { by1 = (int32_t)PANEL_H; }
    box_outline(buf, bx0, by0, bx1, by1, colour);

    if ((det->class_index >= 0) && (det->class_index < NB_CLASSES))
    {
      name = classes_table[det->class_index];
    }
    (void)snprintf(label, sizeof(label), "%s %d%%", name, (int)((det->conf * 100.0f) + 0.5f));
    lh  = (int32_t)Font16.Height + (2 * OVL_LABEL_PAD);
    ly0 = by0 - lh;
    if (ly0 < 0) { ly0 = 0; }
    fill_rect(buf, bx0, ly0,
              bx0 + ((int32_t)strlen(label) * (int32_t)Font16.Width) + (2 * OVL_LABEL_PAD),
              ly0 + lh, OVL_LABEL_BG);
    text(buf, bx0 + OVL_LABEL_PAD, ly0 + OVL_LABEL_PAD, label);
  }

  if ((mode == OVL_RELOAD_CLEAR) || (mode == OVL_RELOAD))
  {
    OVL_Commit();
  }
}

#else /* NIA_LCD */

/* Without the panel there is nothing to draw on. */
void LCD_Overlay_Nia_From(const struct nia_pp_out *out, OVL_Mode_t mode)
{
  (void)out;
  (void)mode;
}

#endif /* NIA_LCD */
