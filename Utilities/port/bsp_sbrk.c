/**
  * @file    bsp_sbrk.c
  * @brief   Minimal heap provider (_sbrk) for bare-metal Neuro N6 applications.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * The heap grows from the end of .bss up to _Min_Stack_Size below _estack. Supply your
  * own _sbrk to place it elsewhere, for example in external PSRAM.
  */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/* Provided by the linker script. */
extern uint8_t _end;          /* first byte after .bss                    */
extern uint8_t _estack;       /* top of stack                             */
extern uint8_t _Min_Stack_Size;

void *_sbrk(ptrdiff_t incr)
{
  static uint8_t *heap = NULL;
  uint8_t        *prev;
  const uint8_t  *limit = (const uint8_t *)(&_estack) - (size_t)(&_Min_Stack_Size);

  if (heap == NULL) { heap = &_end; }

  if ((incr > 0) && ((size_t)(limit - heap) < (size_t)incr))
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev  = heap;
  heap += incr;
  return (void *)prev;
}
