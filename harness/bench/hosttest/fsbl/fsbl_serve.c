/* The simulated board with its bootloader's serial stream on a
 * pseudo-terminal, for a real host tool to upload to. test_uploader.py runs
 * the repository's own uploader against it. See README.md.
 *
 *   fsbl_serve [--load file@offset]... [--dump file@offset:length]...
 *              [--bkp index=value]... [--idle ms] [--boots n] [--app] [--reboot]
 *              [--said]
 *
 * Offsets, lengths and values are hexadecimal. The board starts in the
 * bootloader, as if an application had asked for it, unless --app is given.
 * It prints the terminal's name, then how each boot ended:
 *
 *   PTY /dev/pts/3
 *   END RESET offset=0x00000000 dfu=1 watchdog=0
 *   END APP offset=0x00100000 dfu=0 watchdog=0
 *
 * and stops at the first boot that does not end in a reset, writing the
 * regions asked for with --dump. With --reboot, a bootloader the host has
 * left is followed by a press of the reset button, once, and the board runs
 * on from there. With --said, each boot is followed by what the bootloader
 * sent the host, a SAID line for each of its lines.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsbl_sim.h"

#define FORCEDFU 0x00F4CDF4u

static int load(const char *spec)
{
  char path[512];
  unsigned long off = 0ul;
  FILE *f;
  size_t n;

  if (sscanf(spec, "%511[^@]@%lx", path, &off) != 2) { return -1; }
  f = fopen(path, "rb");
  if (f == NULL) { return -1; }
  n = fread(sim_flash((uint32_t)off), 1u, SIM_FLASH_SIZE - off, f);
  fclose(f);
  return (n > 0u) ? 0 : -1;
}

static int dump(const char *spec)
{
  char path[512];
  unsigned long off = 0ul, len = 0ul;
  FILE *f;

  if (sscanf(spec, "%511[^@]@%lx:%lx", path, &off, &len) != 3) { return -1; }
  if ((off > SIM_FLASH_SIZE) || (len > SIM_FLASH_SIZE - off)) { return -1; }
  f = fopen(path, "wb");
  if (f == NULL) { return -1; }
  fwrite(sim_flash((uint32_t)off), 1u, len, f);
  fclose(f);
  return 0;
}

int main(int argc, char **argv)
{
  char name[128];
  int boots = 4, dfu = 1, reboot = 0, said = 0, end = SIM_END_NONE;
  unsigned long idle = 3000ul;

  sim_init();
  for (int i = 1; i < argc; i++) {
    const char *arg = (i + 1 < argc) ? argv[i + 1] : "";
    unsigned long idx = 0ul, val = 0ul;

    if (strcmp(argv[i], "--app") == 0) {
      dfu = 0;
    } else if (strcmp(argv[i], "--reboot") == 0) {
      reboot = 1;
    } else if (strcmp(argv[i], "--said") == 0) {
      said = 1;
    } else if ((strcmp(argv[i], "--load") == 0) && (load(arg) == 0)) {
      i++;
    } else if (strcmp(argv[i], "--dump") == 0) {
      i++;                                   /* written at the end */
    } else if ((strcmp(argv[i], "--bkp") == 0) && (sscanf(arg, "%lu=%lx", &idx, &val) == 2) && (idx < 32ul)) {
      *sim_bkp((uint32_t)idx) = (uint32_t)val;
      i++;
    } else if ((strcmp(argv[i], "--idle") == 0) && (sscanf(arg, "%lu", &idle) == 1)) {
      i++;
    } else if ((strcmp(argv[i], "--boots") == 0) && (sscanf(arg, "%d", &boots) == 1)) {
      i++;
    } else {
      fprintf(stderr, "fsbl_serve: cannot use %s %s\n", argv[i], arg);
      return 2;
    }
  }
  if (sim_pty_open(name, sizeof(name)) != 0) {
    perror("fsbl_serve: pseudo-terminal");
    return 2;
  }
  sim->dfu_idle_limit_ms = (uint32_t)idle;
  if (dfu) {
    *sim_bkp(2) = FORCEDFU;
  }
  printf("PTY %s\n", name);
  fflush(stdout);

  for (int i = 0; i < boots; i++) {
    end = sim_boot();
    printf("END %s offset=0x%08lX dfu=%d watchdog=%d\n", sim_end_name(end),
           (unsigned long)((end == SIM_END_APP) ? sim->boot_offset : 0u), sim->entered_dfu, sim->wdg_running);
    if (said) {                              /* what the bootloader sent, a line of it to a line */
      for (const char *p = sim->d2h; *p != '\0'; ) {
        const size_t n = strcspn(p, "\r\n");

        if (n > 0u) { printf("SAID %.*s\n", (int)n, p); }
        p += n + strspn(p + n, "\r\n");
      }
    }
    fflush(stdout);
    if ((end == SIM_END_DFU_IDLE) && reboot) {
      reboot = 0;
      (void)sim_elapse(3000u);               /* the button, some seconds later */
      continue;
    }
    if ((end != SIM_END_RESET) && (end != SIM_END_WATCHDOG)) {
      break;
    }
  }
  for (int i = 1; i + 1 < argc; i++) {
    if ((strcmp(argv[i], "--dump") == 0) && (dump(argv[i + 1]) != 0)) {
      fprintf(stderr, "fsbl_serve: cannot dump %s\n", argv[i + 1]);
      return 2;
    }
  }
  return 0;
}
