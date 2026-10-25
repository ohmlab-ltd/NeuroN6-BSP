/**
  * @file    lepton.h
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
  * Interface Description Document. This is an original implementation written
  * against those documents; it is not derived from any other driver.
  */

#ifndef LEPTON_H
#define LEPTON_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Commands: 16-bit registers on I2C, used through get, set and run transactions.
 * Video: a frame is four segments of 60 packets of 80 pixels, with filler packets
 * whenever the reader is ahead. No bus or pin access here: the board supplies the
 * I2C transactions and owns the SPI, the chip select and the control lines. */

/** @brief Command-bus address, 8-bit form. */
#define LEPTON_I2C_ADDRESS              0x54U

/* Command-interface registers */
#define LEPTON_REG_POWER                0x0000U
#define LEPTON_REG_STATUS               0x0002U /*!< Busy, boot and result.    */
#define LEPTON_REG_COMMAND              0x0004U
#define LEPTON_REG_DATA_LENGTH          0x0006U /*!< Words to transfer.        */
#define LEPTON_REG_DATA_0               0x0008U /*!< 16 data registers, two apart. */
#define LEPTON_REG_DATA_BLOCK           0xF800U /*!< Block area (> 16 words), unused. */

/** @brief Data registers reachable without the block area. */
#define LEPTON_DATA_REGISTERS           16U

#define LEPTON_STATUS_BUSY              0x0001U /*!< A command is running.     */
#define LEPTON_STATUS_BOOT_MODE         0x0002U /*!< Booted from internal memory. */
#define LEPTON_STATUS_BOOT_STATUS       0x0004U /*!< Boot finished.            */

/* Command words: module in bits 11:8, command in 7:2, type in 1:0 (0 get, 1 set,
 * 2 run); protected modules also set bit 14. The words below are complete. */
#define LEPTON_CMD_SYS_PING             0x0202U /*!< Run: is the part alive.   */
#define LEPTON_CMD_SYS_STATUS           0x0204U /*!< Get: status, 4 words.     */
#define LEPTON_CMD_SYS_SERIAL           0x0208U /*!< Get: serial, 4 words.     */
#define LEPTON_CMD_SYS_UPTIME           0x020CU /*!< Get: uptime, 2 words.     */
#define LEPTON_CMD_SYS_AUX_TEMP_K       0x0210U /*!< Get: housing temperature. */
#define LEPTON_CMD_SYS_FPA_TEMP_K       0x0214U /*!< Get: sensor temperature.  */
#define LEPTON_CMD_SYS_RUN_FFC          0x0242U /*!< Run: flat-field correction. */
#define LEPTON_CMD_AGC_ENABLE_GET       0x0100U /*!< Get: AGC state, 2 words.  */
#define LEPTON_CMD_AGC_ENABLE_SET       0x0101U /*!< Set: AGC state, 2 words.  */

/* Driver codes, far outside the camera's own results (a small signed number in the
 * status register's top byte, 0 = success), so a bus failure differs from a refusal. */
#define LEPTON_OK                       0
#define LEPTON_ERROR                    -1000  /*!< A transaction failed.      */
#define LEPTON_TIMEOUT                  -1001  /*!< The part stayed busy.      */
#define LEPTON_WRONG_PARAM              -1002  /*!< Argument out of range.     */

/* Video protocol (VoSPI) */
#define LEPTON_WIDTH                    160U
#define LEPTON_HEIGHT                   120U
#define LEPTON_FRAME_PIXELS             (LEPTON_WIDTH * LEPTON_HEIGHT)
#define LEPTON_PACKET_BYTES             164U  /*!< 2 header, 2 CRC, 160 payload. */
#define LEPTON_PACKET_PIXELS            80U
#define LEPTON_PACKETS_PER_SEGMENT      60U
#define LEPTON_SEGMENTS                 4U
#define LEPTON_SEGMENT_ROWS             (LEPTON_HEIGHT / LEPTON_SEGMENTS)

/** @brief The only packet in a segment whose header gives the segment number. */
#define LEPTON_SEGMENT_ID_PACKET        20U

typedef int32_t (*LEPTON_Init_Func)(void);
typedef int32_t (*LEPTON_DeInit_Func)(void);
typedef int32_t (*LEPTON_GetTick_Func)(void);
typedef void    (*LEPTON_Delay_Func)(uint32_t);
typedef int32_t (*LEPTON_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LEPTON_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LEPTON_IsReady_Func)(uint16_t, uint32_t);

/** @brief Bus access the board supplies. */
typedef struct
{
  LEPTON_Init_Func     Init;     /*!< Bring the command bus up.              */
  LEPTON_DeInit_Func   DeInit;   /*!< Release the command bus.               */
  uint16_t             Address;  /*!< Device address, 8-bit form.            */
  LEPTON_WriteReg_Func WriteReg; /*!< Sixteen-bit register write.            */
  LEPTON_ReadReg_Func  ReadReg;  /*!< Sixteen-bit register read.             */
  LEPTON_IsReady_Func  IsReady;  /*!< Address probe, or NULL.                */
  LEPTON_GetTick_Func  GetTick;  /*!< Millisecond tick, for the waits.       */
  LEPTON_Delay_Func    Delay;    /*!< Millisecond delay, for the waits.      */
} LEPTON_IO_t;

/** @brief Driver state for the command interface. */
typedef struct
{
  LEPTON_IO_t IO;            /*!< Bus access supplied by the board.          */
  uint8_t     IsInitialized; /*!< 1 once the part has finished booting.      */
  int16_t     LastResult;    /*!< Result of the most recent transaction.     */
} LEPTON_Object_t;

/** @brief Video parser state, kept across blocks; one per stream. */
typedef struct
{
  uint16_t Segment[LEPTON_PACKETS_PER_SEGMENT * LEPTON_PACKET_PIXELS];
                            /*!< Segment under construction.                 */
  uint32_t ExpectedPacket;  /*!< Packet number the sequence requires next.   */
  int32_t  CurrentSegment;  /*!< Segment number the header announced, or -1. */
  uint32_t SegmentsHeld;    /*!< Segments of this frame copied out, 0 to 3.  */
  uint16_t *pAssembly;      /*!< Frame those segments were copied into.     */
  uint32_t PacketsSeen;     /*!< Well-formed packets parsed, ever.           */
  uint32_t FillersSeen;     /*!< Filler packets parsed, ever.                */
  uint32_t SegmentsParsed;  /*!< Segments completed, ever.                   */
  uint32_t FramesParsed;    /*!< Frames completed, ever.                     */
  uint32_t Resyncs;         /*!< Times the packet sequence was lost, ever.   */
  uint32_t LastHeader;      /*!< First four bytes of the last block parsed: tells
                              *   a dead line from a live one.               */
} LEPTON_VoSPI_t;

/** @brief Copy the board's bus access into pObj; nothing is sent to the part. */
int32_t LEPTON_RegisterBusIO(LEPTON_Object_t *pObj, const LEPTON_IO_t *pIO);

/**
  * @brief  Bring the bus up and wait up to BootTimeout_ms for the part to boot.
  * @note   The boot takes about 950 ms. Polled while it boots (this polls every 20 ms),
  *         or booting on a busy bus, the part never finishes: call this only after a
  *         quiet bus for long enough after the reset release (2 s is enough).
  * @retval LEPTON_ERROR if the bus would not come up, LEPTON_TIMEOUT if the part never
  *         reported a finished boot (the bus is then released).
  */
int32_t LEPTON_Init(LEPTON_Object_t *pObj, uint32_t BootTimeout_ms);

/** @brief Release the command bus. */
int32_t LEPTON_DeInit(LEPTON_Object_t *pObj);

/** @brief Address probe: LEPTON_OK if something answered, else LEPTON_ERROR. */
int32_t LEPTON_IsPresent(LEPTON_Object_t *pObj);

/** @brief Read or write one register; big-endian on the bus, converted here. */
int32_t LEPTON_ReadRegister(LEPTON_Object_t *pObj, uint16_t Reg, uint16_t *pValue);
int32_t LEPTON_WriteRegister(LEPTON_Object_t *pObj, uint16_t Reg, uint16_t Value);

int32_t LEPTON_GetStatus(LEPTON_Object_t *pObj, uint16_t *pStatus);

/** @brief Wait up to Timeout_ms for the part to stop being busy. */
int32_t LEPTON_WaitReady(LEPTON_Object_t *pObj, uint32_t Timeout_ms);

/**
  * @brief  Get an attribute (command word ending in 0) of up to LEPTON_DATA_REGISTERS
  *         words, in the order the part sends them.
  * @retval 0, the camera's small negative refusal, or a LEPTON_* bus or timeout code;
  *         the same for the Set, Run and wrapper calls below.
  */
int32_t LEPTON_GetAttribute(LEPTON_Object_t *pObj, uint16_t Command,
                            uint16_t *pWords, uint16_t LengthWords);

/** @brief Set an attribute (word ending in 1), up to LEPTON_DATA_REGISTERS words. */
int32_t LEPTON_SetAttribute(LEPTON_Object_t *pObj, uint16_t Command,
                            const uint16_t *pWords, uint16_t LengthWords);

/** @brief Run a procedure (command word ending in 2). */
int32_t LEPTON_RunCommand(LEPTON_Object_t *pObj, uint16_t Command);

int32_t LEPTON_Ping(LEPTON_Object_t *pObj);

/** @brief Serial number, four words. */
int32_t LEPTON_GetSerialNumber(LEPTON_Object_t *pObj, uint16_t *pSerial);

/** @brief Uptime in ms. */
int32_t LEPTON_GetUptime(LEPTON_Object_t *pObj, uint32_t *pUptime_ms);

/** @brief Detector (FPA) temperature in 0.01 K. */
int32_t LEPTON_GetFpaTemperature(LEPTON_Object_t *pObj, uint16_t *pKelvin100);

/** @brief Housing temperature in 0.01 K. */
int32_t LEPTON_GetAuxTemperature(LEPTON_Object_t *pObj, uint16_t *pKelvin100);

/** @brief AGC state, 1 or 0; with AGC on, pixels are display values, not temperatures. */
int32_t LEPTON_GetAgcEnabled(LEPTON_Object_t *pObj, uint8_t *pEnabled);

int32_t LEPTON_SetAgcEnabled(LEPTON_Object_t *pObj, uint8_t Enabled);

/** @brief Flat-field correction; video pauses for up to about a second. */
int32_t LEPTON_RunFfc(LEPTON_Object_t *pObj);

/** @brief Return the parser to hunting for a segment start; counters are kept. */
int32_t LEPTON_VoSPI_Reset(LEPTON_VoSPI_t *pParser);

/** @brief Clear the parser completely, counters included. */
int32_t LEPTON_VoSPI_Init(LEPTON_VoSPI_t *pParser);

/**
  * @brief  Parse PacketCount whole packets into pFrame (LEPTON_FRAME_PIXELS, row major).
  * @param  pFrame      Pass the same buffer every call: segments land as they complete,
  *                     so it holds a frame only when this returns 1. A segment completing
  *                     after a frame in the same block is dropped, and its frame lost.
  * @param  pRecognised Well-formed, in-sequence packets, fillers included, or NULL.
  *                     Packet 0 counts only once packet 1 follows (a line held low reads
  *                     as packet 0): a block may count one short, the next one over.
  * @note   Packets are checked by ID and sequence, not CRC. Judge health by the
  *         recognised count, not frames: the part can send only fillers for seconds.
  * @retval 1 if a frame completed, 0 if not, LEPTON_WRONG_PARAM for a NULL argument.
  */
int32_t LEPTON_VoSPI_Parse(LEPTON_VoSPI_t *pParser, const uint8_t *pBlock,
                           uint32_t PacketCount, uint16_t *pFrame,
                           uint32_t *pRecognised);

/** @brief Rotate a frame 180 degrees in place. */
int32_t LEPTON_VoSPI_Rotate180(uint16_t *pFrame);

#ifdef __cplusplus
}
#endif

#endif /* LEPTON_H */
