/**
  * @file    lepton.c
  * @brief   FLIR Lepton 3.1R driver: command interface and video-protocol parser.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * Register addresses, command words, protocol framing and timings are taken
  * from the FLIR Lepton Engineering Datasheet and the Lepton Software
  * Interface Description Document.
  */

#include <string.h>

#include "lepton.h"

/* Longest a command may stay busy, ms. */
#define LEPTON_COMMAND_TIMEOUT_MS   1000U

/* Status poll interval, ms. */
#define LEPTON_POLL_INTERVAL_MS     2U

/* Address probes before giving up. */
#define LEPTON_PROBE_TRIALS         2U

#define LEPTON_PROBE_TIMEOUT_MS     100U

/* Low nibble of the first header byte that marks a filler packet. */
#define LEPTON_FILLER_NIBBLE        0x0FU

static void    LEPTON_Wait(LEPTON_Object_t *pObj, uint32_t Milliseconds);
static int16_t LEPTON_ResultOf(uint16_t Status);
static int32_t LEPTON_ReadData(LEPTON_Object_t *pObj, uint16_t *pWords, uint16_t LengthWords);
static int32_t LEPTON_WriteData(LEPTON_Object_t *pObj, const uint16_t *pWords,
                                uint16_t LengthWords);

static void LEPTON_Wait(LEPTON_Object_t *pObj, uint32_t Milliseconds)
{
  if (pObj->IO.Delay != NULL)
  {
    pObj->IO.Delay(Milliseconds);
  }
}

/* The result is a signed byte in the top half: sign-extend it, do not just shift. */
static int16_t LEPTON_ResultOf(uint16_t Status)
{
  return (int16_t)((int8_t)((Status >> 8) & 0xFFU));
}

/* One register at a time: they are two apart, so a burst would need the block area. */
static int32_t LEPTON_ReadData(LEPTON_Object_t *pObj, uint16_t *pWords, uint16_t LengthWords)
{
  uint16_t index;

  if (LengthWords > LEPTON_DATA_REGISTERS)
  {
    return LEPTON_WRONG_PARAM;
  }

  for (index = 0U; index < LengthWords; index++)
  {
    uint16_t reg = (uint16_t)(LEPTON_REG_DATA_0 + (index * 2U));

    if (LEPTON_ReadRegister(pObj, reg, &pWords[index]) != LEPTON_OK)
    {
      return LEPTON_ERROR;
    }
  }

  return LEPTON_OK;
}

static int32_t LEPTON_WriteData(LEPTON_Object_t *pObj, const uint16_t *pWords,
                                uint16_t LengthWords)
{
  uint16_t index;

  if (LengthWords > LEPTON_DATA_REGISTERS)
  {
    return LEPTON_WRONG_PARAM;
  }

  for (index = 0U; index < LengthWords; index++)
  {
    uint16_t reg = (uint16_t)(LEPTON_REG_DATA_0 + (index * 2U));

    if (LEPTON_WriteRegister(pObj, reg, pWords[index]) != LEPTON_OK)
    {
      return LEPTON_ERROR;
    }
  }

  return LEPTON_OK;
}

int32_t LEPTON_RegisterBusIO(LEPTON_Object_t *pObj, const LEPTON_IO_t *pIO)
{
  if ((pObj == NULL) || (pIO == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  if ((pIO->ReadReg == NULL) || (pIO->WriteReg == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  pObj->IO = *pIO;
  pObj->IsInitialized = 0U;
  pObj->LastResult = LEPTON_OK;

  return LEPTON_OK;
}

int32_t LEPTON_Init(LEPTON_Object_t *pObj, uint32_t BootTimeout_ms)
{
  int32_t start;

  if ((pObj == NULL) || (pObj->IO.GetTick == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  if (pObj->IO.Init != NULL)
  {
    if (pObj->IO.Init() != 0)
    {
      return LEPTON_ERROR;
    }
  }

  start = pObj->IO.GetTick();

  for (;;)
  {
    uint16_t status = 0U;

    if (LEPTON_GetStatus(pObj, &status) == LEPTON_OK)
    {
      if (((status & LEPTON_STATUS_BOOT_STATUS) != 0U) &&
          ((status & LEPTON_STATUS_BUSY) == 0U))
      {
        pObj->IsInitialized = 1U;
        return LEPTON_OK;
      }
    }

    if (((uint32_t)pObj->IO.GetTick() - (uint32_t)start) >= BootTimeout_ms)
    {
      /* No camera, or one that did not boot: the bus goes back. */
      if (pObj->IO.DeInit != NULL)
      {
        (void)pObj->IO.DeInit();
      }

      return LEPTON_TIMEOUT;
    }

    /* 20 ms, not 2: fewer transactions on a bus the booting part needs quiet. */
    LEPTON_Wait(pObj, 20U);
  }
}

int32_t LEPTON_DeInit(LEPTON_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  if (pObj->IO.DeInit != NULL)
  {
    (void)pObj->IO.DeInit();
  }

  pObj->IsInitialized = 0U;

  return LEPTON_OK;
}

int32_t LEPTON_IsPresent(LEPTON_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  if (pObj->IO.IsReady == NULL)
  {
    /* No probe: a register read proves the same. */
    uint16_t status = 0U;

    return LEPTON_ReadRegister(pObj, LEPTON_REG_STATUS, &status);
  }

  if (pObj->IO.IsReady(pObj->IO.Address, LEPTON_PROBE_TRIALS) != 0)
  {
    return LEPTON_ERROR;
  }

  return LEPTON_OK;
}

int32_t LEPTON_ReadRegister(LEPTON_Object_t *pObj, uint16_t Reg, uint16_t *pValue)
{
  uint8_t buffer[2] = {0U, 0U};

  if ((pObj == NULL) || (pValue == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  if (pObj->IO.ReadReg(pObj->IO.Address, Reg, buffer, 2U) != 0)
  {
    pObj->LastResult = LEPTON_ERROR;
    return LEPTON_ERROR;
  }

  *pValue = (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);

  return LEPTON_OK;
}

int32_t LEPTON_WriteRegister(LEPTON_Object_t *pObj, uint16_t Reg, uint16_t Value)
{
  uint8_t buffer[2];

  if (pObj == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  buffer[0] = (uint8_t)(Value >> 8);
  buffer[1] = (uint8_t)(Value & 0xFFU);

  if (pObj->IO.WriteReg(pObj->IO.Address, Reg, buffer, 2U) != 0)
  {
    pObj->LastResult = LEPTON_ERROR;
    return LEPTON_ERROR;
  }

  return LEPTON_OK;
}

int32_t LEPTON_GetStatus(LEPTON_Object_t *pObj, uint16_t *pStatus)
{
  return LEPTON_ReadRegister(pObj, LEPTON_REG_STATUS, pStatus);
}

int32_t LEPTON_WaitReady(LEPTON_Object_t *pObj, uint32_t Timeout_ms)
{
  int32_t start;

  if ((pObj == NULL) || (pObj->IO.GetTick == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  start = pObj->IO.GetTick();

  for (;;)
  {
    uint16_t status = 0U;

    if ((LEPTON_GetStatus(pObj, &status) == LEPTON_OK) &&
        ((status & LEPTON_STATUS_BUSY) == 0U))
    {
      return LEPTON_OK;
    }

    if (((uint32_t)pObj->IO.GetTick() - (uint32_t)start) >= Timeout_ms)
    {
      return LEPTON_TIMEOUT;
    }

    LEPTON_Wait(pObj, LEPTON_POLL_INTERVAL_MS);
  }
}

int32_t LEPTON_GetAttribute(LEPTON_Object_t *pObj, uint16_t Command,
                            uint16_t *pWords, uint16_t LengthWords)
{
  uint16_t status = 0U;
  int16_t  result;

  if ((pObj == NULL) || (LengthWords > LEPTON_DATA_REGISTERS))
  {
    return LEPTON_WRONG_PARAM;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  /* Length before command: it sets how many data registers the part fills. */
  if (LEPTON_WriteRegister(pObj, LEPTON_REG_DATA_LENGTH, LengthWords) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  if (LEPTON_WriteRegister(pObj, LEPTON_REG_COMMAND, Command) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  if (LEPTON_GetStatus(pObj, &status) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  result = LEPTON_ResultOf(status);

  if ((result == 0) && (pWords != NULL) && (LengthWords != 0U))
  {
    if (LEPTON_ReadData(pObj, pWords, LengthWords) != LEPTON_OK)
    {
      return LEPTON_ERROR;
    }
  }

  pObj->LastResult = result;

  return (int32_t)result;
}

int32_t LEPTON_SetAttribute(LEPTON_Object_t *pObj, uint16_t Command,
                            const uint16_t *pWords, uint16_t LengthWords)
{
  uint16_t status = 0U;

  if ((pObj == NULL) || (LengthWords > LEPTON_DATA_REGISTERS))
  {
    return LEPTON_WRONG_PARAM;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  /* Data, then length, then command: the part reads the data when the command lands. */
  if ((pWords != NULL) && (LengthWords != 0U))
  {
    if (LEPTON_WriteData(pObj, pWords, LengthWords) != LEPTON_OK)
    {
      return LEPTON_ERROR;
    }
  }

  if (LEPTON_WriteRegister(pObj, LEPTON_REG_DATA_LENGTH, LengthWords) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  if (LEPTON_WriteRegister(pObj, LEPTON_REG_COMMAND, Command) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  if (LEPTON_GetStatus(pObj, &status) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  pObj->LastResult = LEPTON_ResultOf(status);

  return (int32_t)pObj->LastResult;
}

int32_t LEPTON_RunCommand(LEPTON_Object_t *pObj, uint16_t Command)
{
  uint16_t status = 0U;

  if (pObj == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  if (LEPTON_WriteRegister(pObj, LEPTON_REG_COMMAND, Command) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  if (LEPTON_WaitReady(pObj, LEPTON_COMMAND_TIMEOUT_MS) != LEPTON_OK)
  {
    pObj->LastResult = LEPTON_TIMEOUT;
    return LEPTON_TIMEOUT;
  }

  if (LEPTON_GetStatus(pObj, &status) != LEPTON_OK)
  {
    return LEPTON_ERROR;
  }

  pObj->LastResult = LEPTON_ResultOf(status);

  return (int32_t)pObj->LastResult;
}

int32_t LEPTON_Ping(LEPTON_Object_t *pObj)
{
  return LEPTON_RunCommand(pObj, LEPTON_CMD_SYS_PING);
}

int32_t LEPTON_GetSerialNumber(LEPTON_Object_t *pObj, uint16_t *pSerial)
{
  if (pSerial == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  return LEPTON_GetAttribute(pObj, LEPTON_CMD_SYS_SERIAL, pSerial, 4U);
}

int32_t LEPTON_GetUptime(LEPTON_Object_t *pObj, uint32_t *pUptime_ms)
{
  uint16_t words[2] = {0U, 0U};
  int32_t  result;

  if (pUptime_ms == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  result = LEPTON_GetAttribute(pObj, LEPTON_CMD_SYS_UPTIME, words, 2U);

  if (result == 0)
  {
    /* Least significant word first, unlike the bytes within a word. */
    *pUptime_ms = ((uint32_t)words[1] << 16) | (uint32_t)words[0];
  }

  return result;
}

int32_t LEPTON_GetFpaTemperature(LEPTON_Object_t *pObj, uint16_t *pKelvin100)
{
  if (pKelvin100 == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  return LEPTON_GetAttribute(pObj, LEPTON_CMD_SYS_FPA_TEMP_K, pKelvin100, 1U);
}

int32_t LEPTON_GetAuxTemperature(LEPTON_Object_t *pObj, uint16_t *pKelvin100)
{
  if (pKelvin100 == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  return LEPTON_GetAttribute(pObj, LEPTON_CMD_SYS_AUX_TEMP_K, pKelvin100, 1U);
}

int32_t LEPTON_GetAgcEnabled(LEPTON_Object_t *pObj, uint8_t *pEnabled)
{
  uint16_t words[2] = {0U, 0U};
  int32_t  result;

  if (pEnabled == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  result = LEPTON_GetAttribute(pObj, LEPTON_CMD_AGC_ENABLE_GET, words, 2U);

  if (result == 0)
  {
    *pEnabled = (words[0] != 0U) ? 1U : 0U;
  }

  return result;
}

int32_t LEPTON_SetAgcEnabled(LEPTON_Object_t *pObj, uint8_t Enabled)
{
  uint16_t words[2];

  words[0] = (Enabled != 0U) ? 1U : 0U;
  words[1] = 0U;

  return LEPTON_SetAttribute(pObj, LEPTON_CMD_AGC_ENABLE_SET, words, 2U);
}

int32_t LEPTON_RunFfc(LEPTON_Object_t *pObj)
{
  return LEPTON_RunCommand(pObj, LEPTON_CMD_SYS_RUN_FFC);
}

int32_t LEPTON_VoSPI_Init(LEPTON_VoSPI_t *pParser)
{
  if (pParser == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  (void)memset(pParser, 0, sizeof(LEPTON_VoSPI_t));
  pParser->CurrentSegment = -1;

  return LEPTON_OK;
}

int32_t LEPTON_VoSPI_Reset(LEPTON_VoSPI_t *pParser)
{
  if (pParser == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  pParser->ExpectedPacket = 0U;
  pParser->CurrentSegment = -1;
  pParser->SegmentsHeld   = 0U;
  pParser->pAssembly      = NULL;

  return LEPTON_OK;
}

int32_t LEPTON_VoSPI_Parse(LEPTON_VoSPI_t *pParser, const uint8_t *pBlock,
                           uint32_t PacketCount, uint16_t *pFrame,
                           uint32_t *pRecognised)
{
  uint32_t index;
  uint32_t recognised = 0U;
  int32_t  frame_done = 0;

  if ((pParser == NULL) || (pBlock == NULL) || (pFrame == NULL))
  {
    return LEPTON_WRONG_PARAM;
  }

  if (PacketCount >= 1U)
  {
    pParser->LastHeader = ((uint32_t)pBlock[0] << 24) | ((uint32_t)pBlock[1] << 16) |
                          ((uint32_t)pBlock[2] << 8)  | (uint32_t)pBlock[3];
  }

  for (index = 0U; index < PacketCount; index++)
  {
    const uint8_t *packet = &pBlock[index * LEPTON_PACKET_BYTES];
    const uint8_t *payload;
    uint16_t      *destination;
    uint32_t       number;
    uint32_t       pixel;
    int32_t        segment;

    /* Fillers (the reader is ahead, by design) can come anywhere, even mid-segment,
     * and do not break the sequence. */
    if ((packet[0] & 0x0FU) == LEPTON_FILLER_NIBBLE)
    {
      recognised++;
      pParser->FillersSeen++;
      continue;
    }

    number = (uint32_t)packet[1];

    /* The packet number is the ID's low 12 bits and at most 59: bits 11:8 are clear. */
    if (((packet[0] & 0x0FU) != 0U) || (number >= LEPTON_PACKETS_PER_SEGMENT))
    {
      /* Impossible header: misaligned or damaged. Back to hunting. */
      pParser->ExpectedPacket = 0U;
      pParser->SegmentsHeld = 0U;
      pParser->Resyncs++;
      continue;
    }

    pParser->PacketsSeen++;

    if (pParser->ExpectedPacket == 0U)
    {
      if (number != 0U)
      {
        recognised++;
        continue;   /* Still hunting for the start of a segment. */
      }

      /* Packet 0 counts only once packet 1 follows: a line held low reads as packet 0. */
      pParser->CurrentSegment = -1;
    }
    else if (number != pParser->ExpectedPacket)
    {
      /* Out of sequence, and not counted as well formed. */
      pParser->ExpectedPacket = 0U;
      pParser->SegmentsHeld = 0U;
      pParser->Resyncs++;
      continue;
    }
    else
    {
      /* Packet 1 also counts for packet 0. */
      recognised += (number == 1U) ? 2U : 1U;
    }

    if (number == LEPTON_SEGMENT_ID_PACKET)
    {
      /* The only packet in the segment that says which segment it is. */
      pParser->CurrentSegment = (int32_t)((packet[0] >> 4) & 0x07U);
    }

    payload = &packet[4];
    destination = &pParser->Segment[number * LEPTON_PACKET_PIXELS];

    for (pixel = 0U; pixel < LEPTON_PACKET_PIXELS; pixel++)
    {
      destination[pixel] = (uint16_t)(((uint16_t)payload[2U * pixel] << 8) |
                                      (uint16_t)payload[(2U * pixel) + 1U]);
    }

    pParser->ExpectedPacket = number + 1U;

    if (pParser->ExpectedPacket < LEPTON_PACKETS_PER_SEGMENT)
    {
      continue;
    }

    pParser->ExpectedPacket = 0U;
    pParser->SegmentsParsed++;

    segment = pParser->CurrentSegment;

    if ((segment < 1) || (segment > (int32_t)LEPTON_SEGMENTS))
    {
      /* Segment 0 is filler between frames: not an error, and progress is kept. */
      continue;
    }

    if (segment == 1)
    {
      pParser->SegmentsHeld = 0U;
    }
    else if (((uint32_t)segment != (pParser->SegmentsHeld + 1U)) ||
             (pFrame != pParser->pAssembly))
    {
      /* Out of order, or earlier segments in another buffer: wait for the next frame. */
      pParser->SegmentsHeld = 0U;
      continue;
    }
    else
    {
      /* In order: fall through and copy it out. */
    }

    if (frame_done != 0)
    {
      /* pFrame holds an undelivered frame: a later segment in this block would overwrite
       * it, so that segment and its frame are dropped. */
      pParser->SegmentsHeld = 0U;
      continue;
    }

    (void)memcpy(&pFrame[((uint32_t)segment - 1U) * LEPTON_SEGMENT_ROWS * LEPTON_WIDTH],
                 pParser->Segment, sizeof(pParser->Segment));

    pParser->pAssembly    = pFrame;
    pParser->SegmentsHeld = (uint32_t)segment;

    if ((uint32_t)segment == LEPTON_SEGMENTS)
    {
      pParser->SegmentsHeld = 0U;
      pParser->FramesParsed++;
      frame_done = 1;
    }
  }

  if (pRecognised != NULL)
  {
    *pRecognised = recognised;
  }

  return frame_done;
}

int32_t LEPTON_VoSPI_Rotate180(uint16_t *pFrame)
{
  uint32_t low;
  uint32_t high;

  if (pFrame == NULL)
  {
    return LEPTON_WRONG_PARAM;
  }

  /* 180 degrees reverses the array: swap from both ends in place. */
  low = 0U;
  high = LEPTON_FRAME_PIXELS - 1U;

  while (low < high)
  {
    uint16_t swap = pFrame[low];

    pFrame[low] = pFrame[high];
    pFrame[high] = swap;

    low++;
    high--;
  }

  return LEPTON_OK;
}
