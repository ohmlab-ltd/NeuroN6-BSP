/* The pseudo-terminal a real host tool opens as the simulated board's serial
 * port. In a file of its own because <termios.h> and the device header both
 * define CR1, CR2 and CR3.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

/* Open a raw, non-blocking pty and give its name; returns the loader's end,
 * or -1. */
int sim_pty_master(char *name, unsigned long size)
{
  struct termios t;
  const int fd = posix_openpt(O_RDWR | O_NOCTTY);

  if ((fd < 0) || (grantpt(fd) != 0) || (unlockpt(fd) != 0) || (ptsname_r(fd, name, size) != 0)) {
    return -1;
  }
  if (tcgetattr(fd, &t) == 0) {
    cfmakeraw(&t);
    (void)tcsetattr(fd, TCSANOW, &t);
  }
  (void)fcntl(fd, F_SETFL, O_NONBLOCK);
  return fd;
}
