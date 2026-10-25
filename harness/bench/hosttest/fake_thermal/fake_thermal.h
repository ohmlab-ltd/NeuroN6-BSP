/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * The board under neuron6_thermal.c in the thermal host tests: a clock that
 * moves only when something waits; the camera's power and reset lines; a
 * camera on the sensor bus; and a video link that records each start of a
 * transfer and how long the chip select was high before it.
 *
 * The camera does not finish booting while anything uses its I2C bus. The
 * fake is stricter: any transaction in the first BOOT_DISTURBED_MS after the
 * reset is released stops it booting until it is powered up again; left
 * alone, it reports a finished boot from BOOT_TAKES_MS on. */
#ifndef FAKE_THERMAL_H
#define FAKE_THERMAL_H

#include <stdint.h>

#define BOOT_DISTURBED_MS  1500U   /* a transaction before this: never boots */
#define BOOT_TAKES_MS       950U   /* left alone, booted from this on       */

/* The clock, in milliseconds, and how far each reading of it moves it on:
 * the first tick_slow readings 1 ms each, every one after tick_step. */
extern uint32_t now;
extern uint32_t tick_slow;
extern uint32_t tick_step;

/* The camera's lines, and its boot. */
extern int      power_high, reset_high;
extern int      power_ups;            /* reset released with power on        */
extern uint32_t released_at;
extern int      camera_on;            /* powered and out of reset            */
extern int      present;              /* it answers on the bus at all        */
extern int      disturbed;            /* a transaction too early in the boot */
extern int      transactions;
extern uint32_t first_transaction_at;

/* The video link. */
extern int      captures;             /* transfers started                   */
extern uint32_t idle_before_capture;  /* ms the select was high before the last */
extern uint8_t *capture_buffer;
extern uint16_t capture_bytes;

/* Back to a board just reset, the clock at Start. The module's own state is
 * the test's to reset, through the BSP calls. */
void fake_reset(uint32_t Start);

#endif /* FAKE_THERMAL_H */
