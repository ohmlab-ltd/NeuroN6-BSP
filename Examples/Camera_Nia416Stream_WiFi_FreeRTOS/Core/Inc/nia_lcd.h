/*
  nia_lcd.h

  The panel, when NIA_LCD is 1: the camera on the TFT-W display carrier's
  800 x 480 panel, with the detector's boxes drawn over it.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: MIT
  This software is licensed under the terms in
  Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
*/

#ifndef NIA_LCD_H
#define NIA_LCD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the panel for a centred CamW x CamH picture, at most the panel's size.
 * Returns the RGB565 buffer the dump pipe writes and the panel shows, or NULL. */
uint8_t *NiaLcd_Init(uint32_t CamW, uint32_t CamH);

/* The overlay buffer to draw the next boxes into; NULL before NiaLcd_Init. */
uint16_t *OVL_GetBackBuffer(void);
/* Show the drawn overlay at the panel's next vertical blank. */
void      OVL_Commit(void);

/* Since start: overlays drawn, and draws that waited for the panel. */
uint32_t  NiaLcd_Draws(void);
uint32_t  NiaLcd_Stalls(void);

#ifdef __cplusplus
}
#endif

#endif /* NIA_LCD_H */
