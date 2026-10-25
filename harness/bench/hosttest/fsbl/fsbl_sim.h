/* A simulated Neuro N6 on a PC for FSBL/Core/Src/main.c, compiled unchanged:
 * external flash at the command level, PSRAM, backup registers, watchdog,
 * clock and the USB serial stream. Each boot runs in its own process; the
 * flash and the backup domain carry over. See README.md.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef FSBL_SIM_H
#define FSBL_SIM_H

#include <stdint.h>

#define SIM_FLASH_BASE   0x90000000ul
#define SIM_FLASH_SIZE   0x02000000ul      /* 32 MB */
#define SIM_PSRAM_BASE   0x70000000ul
#define SIM_PSRAM_SIZE   0x02000000ul      /* 32 MB */

/* How a run of the loader ended. */
enum {
  SIM_END_NONE = 0,
  SIM_END_APP,            /* jumped to an application: boot_offset says where */
  SIM_END_RESET,          /* reset itself */
  SIM_END_DFU_IDLE,       /* in the bootloader with the host done or silent */
  SIM_END_ERROR,          /* Error_Handler: interrupts off, spinning, and no watchdog to end it */
  SIM_END_HANG,           /* stopped making progress */
  SIM_END_WATCHDOG,       /* the watchdog reset it */
  SIM_END_POWER_CUT,      /* the test cut the power (cut_at_op) */
  SIM_END_CRASH,          /* the loader's code faulted on the PC */
};

/* The NOR's worst-case times (MX25UM51245G), so a watchdog is tried against them. */
#define SIM_MS_ERASE_4K    400u
#define SIM_MS_ERASE_64K  2000u
#define SIM_MS_PROGRAM       1u

#define SIM_OPLOG 8192

typedef struct {
  /* How the last run ended. */
  int      end;
  uint32_t boot_offset;        /* flash offset the loader booted from */
  int      boot_sane;          /* that image's stack and reset vector make sense */
  int      entered_dfu;        /* the loader brought USB up */
  uint32_t boots;              /* runs of the loader since sim_init */

  /* The board. */
  uint64_t now_ms;             /* time since sim_init */
  uint64_t rtc_ms;             /* the RTC: time since the backup domain powered */
  uint32_t tick_ms;            /* the HAL tick: time since this boot */
  int      nor_opi;            /* the NOR is in octal DTR mode */
  int      nor_wel;            /* its write-enable latch */
  int      wdg_running;
  uint32_t wdg_timeout_ms;
  uint64_t wdg_deadline_ms;
  int      app_feeds_wdg;      /* the application running now services the watchdog */
  uint32_t wdg_resets;         /* times the watchdog has reset the board */

  /* What a test can break. */
  int      fail_map_call[2];   /* this EXTMEM map-enable call of the boot fails (1 = first) */
  int      fail_map_run[2];    /* and this many after it in a row (0: that one only) */
  int      map_calls[2];
  uint32_t cut_at_op;          /* cut the power after this flash erase/program, 0 never */
  int      boot_fails;         /* BOOT_Application returns an error */
  int      io1_high;           /* the level of a data line nothing drives */
  int      bb_deaf;            /* the NOR does not take the mode change sent at start-up */
  int      nor_locked;         /* the NOR reads, but does not take write enable */
  uint32_t flip_at_program;    /* this page program leaves one bit wrong, 0 never */
  uint32_t program_ms;         /* a page program takes this long, 0 the part's worst case */
  uint32_t dfu_idle_limit_ms;  /* how long the bootloader may sit idle before the run ends */

  /* What happened to the flash. */
  uint32_t ops;                /* erases and programs, in order */
  uint32_t erases, programs;
  uint32_t spi_in_opi;         /* single-line commands sent to an octal flash */
  uint32_t prog_dirty;         /* bytes programmed that were not erased first */
  uint32_t no_wel;             /* erases or programs without write enable */
  struct { uint8_t kind; uint32_t addr; uint32_t len; } oplog[SIM_OPLOG];

  /* The USB serial stream. */
  uint32_t h2d_len, h2d_pos, h2d_released;
  uint32_t frame_end[4096];    /* host frames: where each ends in h2d */
  uint32_t frames, frames_released;
  uint32_t acks, acks_at_release;
  uint64_t release_ms;
  uint32_t d2h_len;
  uint8_t  h2d[8u << 20];
  char     d2h[1u << 20];
} sim_t;

extern sim_t *sim;

/* The test's side. */
void     sim_init(void);                      /* a new board: blank flash, powered */
void     sim_power_cycle(void);               /* off and on: backup domain and PSRAM lost */
int      sim_boot(void);                      /* one run of the loader */
int      sim_run(int max_boots);              /* runs through resets; the last end */
int      sim_elapse(uint32_t ms);             /* the application runs this long; 1 if the
                                                 watchdog reset the board first */
void     sim_ops_clear(void);                 /* forget the flash operations so far */

uint8_t *sim_flash(uint32_t off);
uint8_t *sim_psram(uint32_t addr);
volatile uint32_t *sim_bkp(uint32_t idx);
uint32_t sim_crc32(const void *p, uint32_t len);

void     sim_host_clear(void);
void     sim_host_frame(const void *payload, uint32_t len);   /* one framed command or data block */
void     sim_host_raw(const void *bytes, uint32_t len);       /* bytes as they are */

/* A real host in place of the queued one: the loader's serial stream on a
 * pseudo-terminal, whose name is returned for a host tool to open as a
 * serial port. The bootloader's idle limit then runs in real time. */
int      sim_pty_open(char *name, unsigned long size);
const char *sim_end_name(int end);
int      sim_reply_has(const char *line);     /* the loader sent this line */
int      sim_reply_count(const char *line);

uint32_t sim_erased_in(uint32_t off, uint32_t len);    /* erases that touched this range */
uint32_t sim_programs_in(uint32_t off, uint32_t len);

#endif /* FSBL_SIM_H */
