/**
  * @file    sd_diskio.c
  * @brief   FatFs disk layer on neuron6_sd: drive 0 is the SD card.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "ff.h"
#include "diskio.h"
#include "neuron6_sd.h"

#define SD_DISKIO_DRIVE   0U

static DSTATUS SdDiskio_Status = STA_NOINIT;

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != SD_DISKIO_DRIVE)
  {
    return STA_NOINIT;
  }

  if (BSP_SD_IsDetected() == 0)
  {
    SdDiskio_Status = STA_NOINIT | STA_NODISK;
  }

  return SdDiskio_Status;
}

/* Brings the card up again after a removal or an error. */
DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != SD_DISKIO_DRIVE)
  {
    return STA_NOINIT;
  }

  (void)BSP_SD_DeInit();
  SdDiskio_Status = (BSP_SD_Init() == BSP_ERROR_NONE) ? 0U : STA_NOINIT;

  return SdDiskio_Status;
}

/* FatFs's buffers need not be aligned: neuron6_sd bounces them if need be. */
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  if ((pdrv != SD_DISKIO_DRIVE) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SdDiskio_Status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if (BSP_SD_ReadBlocks(buff, (uint32_t)sector, (uint32_t)count) != BSP_ERROR_NONE)
  {
    /* neuron6_sd resets after an error: the next mount brings the card up again. */
    SdDiskio_Status = STA_NOINIT;
    return RES_ERROR;
  }

  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  if ((pdrv != SD_DISKIO_DRIVE) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SdDiskio_Status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if (BSP_SD_WriteBlocks(buff, (uint32_t)sector, (uint32_t)count) != BSP_ERROR_NONE)
  {
    SdDiskio_Status = STA_NOINIT;
    return RES_ERROR;
  }

  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  BSP_SD_CardInfo_t info;

  if (pdrv != SD_DISKIO_DRIVE)
  {
    return RES_PARERR;
  }

  if ((SdDiskio_Status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    /* Every write has finished programming by the time it returns. */
    case CTRL_SYNC:
      return RES_OK;

    case GET_SECTOR_COUNT:
      if (BSP_SD_GetCardInfo(&info) != BSP_ERROR_NONE)
      {
        return RES_ERROR;
      }
      *(LBA_t *)buff = (LBA_t)info.BlockCount;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = (WORD)BSP_SD_BLOCK_SIZE;
      return RES_OK;

    case GET_BLOCK_SIZE:
      if (BSP_SD_GetCardInfo(&info) != BSP_ERROR_NONE)
      {
        return RES_ERROR;
      }
      *(DWORD *)buff = (DWORD)info.EraseBlocks;
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
