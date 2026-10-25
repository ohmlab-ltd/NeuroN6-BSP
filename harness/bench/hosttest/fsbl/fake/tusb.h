/* Stand-in for TinyUSB: the five calls the boot loader makes, answered by
 * the simulated board (fsbl_sim.c) from a byte stream a test supplies.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef FSBL_SIM_TUSB_H
#define FSBL_SIM_TUSB_H

#include <stdbool.h>
#include <stdint.h>

bool     tusb_init(void);
void     tud_task(void);
uint32_t tud_cdc_n_available(uint8_t itf);
uint32_t tud_cdc_n_read(uint8_t itf, void *buffer, uint32_t bufsize);
uint32_t tud_cdc_n_write(uint8_t itf, const void *buffer, uint32_t bufsize);
uint32_t tud_cdc_n_write_flush(uint8_t itf);

#endif /* FSBL_SIM_TUSB_H */
