/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : USB first-stage boot loader (FSBL)
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * Copyright (c) 2025 STMicroelectronics: STM32CubeMX initialisation, BSD-3-Clause.
 * SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 ******************************************************************************
 */

#include "main.h"
#include "extmem_manager.h"
#include "stm32n6xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "tusb_config.h"
#include "tusb.h"

#include "n6_ota_layout.h"   /* OTA A/B slots + control block (shared with the app) */

/* True once tusb_init() has run; flash_crc32_region() services USB only then,
 * since the boot path CRC-checks slots before USB exists. */
static _Bool g_usb_up = false;

#define MAGIC    0xFE4ADE
#define FORCEDFU 0xF4CDF4  // Backup-register key that forces DFU entry.

/* Bit-banged SPI pins used before the flash is switched into Octal DTR mode. */
#define FLASH_GPIO        GPION
#define FLASH_CS_GPIO     FLASH_GPIO
#define FLASH_CS_PIN      GPIO_PIN_1
#define FLASH_CLK_GPIO    FLASH_GPIO
#define FLASH_CLK_PIN     GPIO_PIN_6
#define FLASH_MOSI_GPIO   FLASH_GPIO
#define FLASH_MOSI_PIN    GPIO_PIN_2
#define FLASH_MISO_GPIO   FLASH_GPIO
#define FLASH_MISO_PIN    GPIO_PIN_3

#define FLASH_CMD_WREN   0x06
#define FLASH_CMD_WRCR2  0x72
#define FLASH_CMD_RDSR   0x05
#define FLASH_CR2_ADDR0_A1  0x00
#define FLASH_CR2_ADDR0_A2  0x00
#define FLASH_CR2_ADDR0_A3  0x00
#define FLASH_CR2_ADDR0_A4  0x00
#define FLASH_CR2_DTR_OPI   0x02

#define MX25UM_PAGE_SIZE       256U
#define APP_XSPI_BASE_DEFAULT  0x00100000u  // Default XIP image location in flash.
#define FLASH_SIZE             (32 * 1024 * 1024UL)  // 32 MB total flash size.
#define MAX_APP_SIZE           (1536 * 1024UL)  // 1.5 MB, one OTA slot (n6_ota_layout.h).
#define XSPI_SECTOR_SIZE       0x1000  // 4 KB

#define OCTAL_WRITE_ENABLE_CMD        0x06F9
#define OCTAL_READ_STATUS_REG_CMD     0x05FA
#define OCTAL_SECTOR_ERASE_CMD        0x21DE
#define OCTAL_BLOCK_ERASE_64K_CMD     0xDC23  // 64 KB erase.
#define OCTAL_PAGE_PROG_CMD           0x12ED
#define OCTAL_IO_DTR_READ_CMD         0xEE11
#define WRITE_ENABLE_CMD        0x06
#define READ_STATUS_REG_CMD     0x05
#define WRITE_CFG_REG_2_CMD     0x72

#define DUMMY_CLOCK_CYCLES_READ_OCTAL   8U  // Dummy cycles for Octal DTR reads.

#define AUTO_POLLING_INTERVAL           0x10U

#define WRITE_ENABLE_MASK_VALUE         0x02U  // Status-register bit masks.
#define WRITE_ENABLE_MATCH_VALUE        0x02U
#define MEMORY_READY_MASK_VALUE         0x01U
#define MEMORY_READY_MATCH_VALUE        0x00U


#define BLOCK_64K_SIZE               0x10000U

XSPI_HandleTypeDef hxspi1;
XSPI_HandleTypeDef hxspi2;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim1;
PCD_HandleTypeDef hpcd_USB_OTG_HS1;

/* System configuration functions. */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_XSPI1_Init(void);
static void MX_XSPI2_Init(void);
static void MX_RTC_Init(void);
void MX_TIM1_Init(void);
static void MX_USB1_OTG_HS_PCD_Init(void);

/* USB functions. */
static void cdc_task(void);
void usb_log(char *msg);
_Bool validate_packet(uint8_t *buf, int len);
static void dfu_feed_data(uint8_t *data, uint32_t len);
/* Flash bit-bang functions. */
static void FLASH_BB_SendByte(uint8_t out);
static uint8_t FLASH_BB_ReadByte(void);
static void FLASH_BB_Init(void);
void FLASH_BB_EnterDTR_OPI(void);
static bool try_get_one_frame(void);
/* Flash OSPI functions. */
void XSPI_EraseAppArea(XSPI_HandleTypeDef *hxspi);
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi);
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
static void XSPI_WritePage(uint32_t address, uint8_t *data, uint32_t len);

static int PSRAM_GlobalReset(XSPI_HandleTypeDef *hxspi);
static int PSRAM_WriteCR(XSPI_HandleTypeDef *hxspi, uint8_t cr, uint32_t addr);

#define RX_BUF_SIZE    8192  // Must be >= the host payload plus framing bytes.
static uint8_t rx_buf[RX_BUF_SIZE];
static uint32_t rx_len = 0;

static uint8_t frame[RX_BUF_SIZE];  // Holds one decoded frame.
static uint32_t frame_len = 0;
uint32_t app_erase_size;
uint32_t app_base_addr = APP_XSPI_BASE_DEFAULT;  // Base address requested by the host.
_Bool dfu_start = false;  // Set after DFU START until the erase/data phase begins.
_Bool dfu_data_mode = false;  // Set while firmware data frames are being accepted.

uint32_t app_write_addr = APP_XSPI_BASE_DEFAULT;  // Current flash write address.
#define FLASH_PAGE_SIZE 256
#define ACK_EVERY_BYTES 16384  // Periodic ACK interval during bulk data writes.

static uint8_t page_buf[FLASH_PAGE_SIZE];
static uint32_t page_fill = 0;
static uint32_t bytes_since_ack = 0;


static inline uint16_t frame_payload_len(void) {
  return (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
}
static inline uint8_t* frame_payload_ptr(void) {
  return &frame[3];
}

/* ---------------------------------------------------------------------------
 * v2 protocol helpers (DFU_INFO / DFU_CRC32 / DFU_NEXT / DFU_RESET)
 * ------------------------------------------------------------------------- */

/* Chip UID (96-bit), also used by usb_descriptors.c for the iSerial. The
 * UID words live in BSEC (UID_BASE = BSEC_BASE + 0x14); the BSEC kernel
 * clock must be enabled to read them. */
void n6_read_uid_words(uint32_t out[3])
{
	__HAL_RCC_BSEC_CLK_ENABLE();
	volatile uint32_t const *uid = (volatile uint32_t const *)UID_BASE;
	for (int w = 0; w < 3; w++) {
		out[w] = uid[w];
	}
}

/* CRC-32 (IEEE 802.3, reflected 0xEDB88320, init/xorout 0xFFFFFFFF), equal to
 * Python's zlib.crc32 in the host tools. The nibble table is 64 bytes. */
static const uint32_t crc32_nibble_tab[16] = {
	0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
	0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
	0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
	0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
};

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t len)
{
	while (len--) {
		crc ^= *p++;
		crc = (crc >> 4) ^ crc32_nibble_tab[crc & 0x0F];
		crc = (crc >> 4) ^ crc32_nibble_tab[crc & 0x0F];
	}
	return crc;
}

/* Copy `len` bytes at flash offset `offset` into RAM through the XSPI1 mapping,
 * then unmap. Returns 0, or -1 if mapping fails. */
static int flash_read_region(uint32_t offset, uint8_t *dst, uint32_t len)
{
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_ENABLE)) {
		HAL_XSPI_Abort(&hxspi1);
		return -1;
	}
	memcpy(dst, (const void *)(0x90000000u + offset), len);
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_DISABLE)) {
		HAL_XSPI_Abort(&hxspi1);
	}
	return 0;
}

/* CRC-32 of `len` bytes at flash offset `offset`, read through the XSPI1
 * mapping. Returns 0 and sets *out, or -1. */
static int flash_crc32_region(uint32_t offset, uint32_t len, uint32_t *out)
{
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_ENABLE)) {
		HAL_XSPI_Abort(&hxspi1);   /* best effort back to indirect mode */
		return -1;
	}

	/* Bounce through a RAM buffer: memcpy issues word loads, so the uncached
	 * mapping serves bursts instead of one flash transaction per byte. */
	static uint8_t crc_buf[4096];
	uint32_t crc = 0xFFFFFFFFu;
	const uint8_t *p = (const uint8_t *)(0x90000000u + offset);
	uint32_t remaining = len;
	uint32_t since_usb = 0;
	while (remaining) {
		uint32_t slice = (remaining > sizeof(crc_buf)) ? sizeof(crc_buf)
		                                               : remaining;
		memcpy(crc_buf, p, slice);
		crc = crc32_update(crc, crc_buf, slice);
		p += slice;
		remaining -= slice;
		since_usb += slice;
		if (g_usb_up && since_usb >= 65536u) {  /* keep USB serviced during long scans */
			since_usb = 0;
			tud_task();
		}
	}
	if (g_usb_up) tud_task();

	/* Leave memory-mapped mode so subsequent erase/program commands work. */
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_DISABLE)) {
		HAL_XSPI_Abort(&hxspi1);
	}

	*out = crc ^ 0xFFFFFFFFu;
	return 0;
}

/* ===========================================================================
 * OTA A/B slot arbitration, layout in n6_ota_layout.h. Without a valid
 * control block, slot A boots.
 * ------------------------------------------------------------------------- */

/* Erase one 4 KB sector at `offset`. Identical command configuration to
 * XSPI_EraseAppArea's sector branch, without the app_base_addr coupling. */
static void ota_erase_4k(uint32_t offset)
{
	XSPI_RegularCmdTypeDef sCommand = { 0 };

	XSPI_WriteEnable(&hxspi1);

	sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.Instruction        = OCTAL_SECTOR_ERASE_CMD;
	sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
	sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
	sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
	sCommand.Address            = offset;
	sCommand.DataMode           = HAL_XSPI_DATA_NONE;
	sCommand.DummyCycles        = 0;
	sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

	if (HAL_XSPI_Command(&hxspi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}
	XSPI_AutoPollingMemReady(&hxspi1);
}

/* Read both control-block copies. Returns the index (0/1) of the winning copy
 * and fills *out, or -1 if neither is valid. */
static int ota_cb_load(n6_ota_cb_t *out)
{
	n6_ota_cb_t copy[2];

	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_ENABLE)) {
		HAL_XSPI_Abort(&hxspi1);
		return -1;
	}
	memcpy(&copy[0], (const void *)(0x90000000u + N6_OTA_CB_OFF(0)), sizeof(n6_ota_cb_t));
	memcpy(&copy[1], (const void *)(0x90000000u + N6_OTA_CB_OFF(1)), sizeof(n6_ota_cb_t));
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(0, EXTMEM_DISABLE)) {
		HAL_XSPI_Abort(&hxspi1);
	}

	int v0 = n6_ota_cb_valid(&copy[0]);
	int v1 = n6_ota_cb_valid(&copy[1]);
	if (!v0 && !v1) return -1;

	int win = (v0 && v1) ? ((copy[1].seq > copy[0].seq) ? 1 : 0) : (v1 ? 1 : 0);
	*out = copy[win];
	return win;
}

/* Write the control block, with seq incremented, to the copy that did not
 * supply it, so a power cut mid-write leaves the other copy intact.
 * `from_copy` is ota_cb_load's return; -1 writes copy 0. */
static void ota_cb_store(n6_ota_cb_t *cb, int from_copy)
{
	int target = (from_copy == 0) ? 1 : 0;

	cb->magic = N6_OTA_CB_MAGIC;
	cb->seq  += 1u;
	cb->crc   = n6_ota_cb_crc(cb);

	ota_erase_4k(N6_OTA_CB_OFF(target));
	XSPI_WritePage(N6_OTA_CB_OFF(target), (uint8_t *)cb, sizeof(*cb));
}

static void ota_bkp_write(uint32_t idx, uint32_t value);

/* An upload starting at a slot and fitting in it cancels any staged update or
 * asset, and becomes the active, confirmed slot if a control block exists. */
static void ota_dfu_note_image(uint32_t base, uint32_t size)
{
	int slot = n6_ota_slot_of_off(base);
	if (slot < 0 || size == 0u) return;
	if (base != n6_ota_slot_off((uint8_t)slot)) return;   /* not at the slot start */
	if (size > N6_OTA_SLOT_SIZE) return;

	ota_bkp_write(N6_OTA_TRIGGER_BKPR, 0u);
	ota_bkp_write(N6_ASSET_TRIGGER_BKPR, 0u);

	n6_ota_cb_t cb;
	int from = ota_cb_load(&cb);
	if (from < 0) return;                                  /* no control block */

	uint32_t crc;
	if (flash_crc32_region(base, size, &crc) != 0) return;

	cb.active                  = (uint8_t)slot;
	cb.pending                 = N6_OTA_NO_PENDING;
	cb.tries_left              = 0u;
	cb.slot[slot].crc32        = crc;
	cb.slot[slot].size         = size;
	cb.slot[slot].state        = N6_OTA_SLOT_CONFIRMED;
	/* A USB upload carries no version, so the slot's version is kept. */

	ota_cb_store(&cb, from);
	usb_log("OTA_CB_UPDATED\r\n");
}

/* Validate a slot the control block tracks: a sane record, then a flash CRC
 * over the recorded size. */
static _Bool ota_slot_ok(const n6_ota_cb_t *cb, uint8_t slot)
{
	uint32_t crc;

	if (slot > 1u) return false;
	if (cb->slot[slot].state == N6_OTA_SLOT_EMPTY) return false;
	if (cb->slot[slot].size == 0u || cb->slot[slot].size > N6_OTA_SLOT_SIZE) return false;
	if (flash_crc32_region(n6_ota_slot_off(slot), cb->slot[slot].size, &crc) != 0) return false;
	return crc == cb->slot[slot].crc32;
}

/* ---- OTA: apply an image staged in PSRAM by the application --------------
 * Flash is untouched until the staged image passes its CRC, only the inactive
 * slot is written, and it is marked VALID only after it reads back correctly. */

static uint32_t ota_bkp_read(uint32_t idx)
{
	return (&TAMP->BKP0R)[idx];
}

static void ota_bkp_write(uint32_t idx, uint32_t value)
{
	HAL_PWR_EnableBkUpAccess();
	(&TAMP->BKP0R)[idx] = value;
	__DSB();
}

static void ota_erase_64k(uint32_t offset)
{
	XSPI_RegularCmdTypeDef sCommand = { 0 };

	XSPI_WriteEnable(&hxspi1);

	sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.Instruction        = OCTAL_BLOCK_ERASE_64K_CMD;
	sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
	sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
	sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
	sCommand.Address            = offset;
	sCommand.DataMode           = HAL_XSPI_DATA_NONE;
	sCommand.DummyCycles        = 0;
	sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

	if (HAL_XSPI_Command(&hxspi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}
	XSPI_AutoPollingMemReady(&hxspi1);
}

/* The running image confirmed itself through N6_OTA_CONFIRM_BKPR: commit the
 * pending slot and stop counting tries. */
static void ota_commit_if_confirmed(void)
{
	if (ota_bkp_read(N6_OTA_CONFIRM_BKPR) != N6_OTA_CONFIRM_MAGIC) {
		return;
	}
	ota_bkp_write(N6_OTA_CONFIRM_BKPR, 0u);

	n6_ota_cb_t cb;
	int from = ota_cb_load(&cb);
	if (from < 0) {
		return;                      /* no control block */
	}
	if (cb.pending == N6_OTA_NO_PENDING || cb.pending > 1u) {
		return;                      /* already committed */
	}

	cb.active                = cb.pending;
	cb.slot[cb.active].state = N6_OTA_SLOT_CONFIRMED;
	cb.pending               = N6_OTA_NO_PENDING;
	cb.tries_left            = 0u;
	ota_cb_store(&cb, from);
}

/* Record an apply's outcome (N6_OTA_APPLY_*) for the application to read. */
static void ota_apply_note(uint32_t code)
{
	ota_bkp_write(N6_OTA_APPLY_BKPR, code);
}

/* Leave the PSRAM unmapped: the application's XSPI2 setup is ignored while the
 * peripheral is memory mapped. */
static void ota_apply_done(void)
{
	(void)EXTMEM_MemoryMappedMode(1, EXTMEM_DISABLE);
}

static void ota_apply_from_psram(void)
{
	if (ota_bkp_read(N6_OTA_TRIGGER_BKPR) != N6_OTA_TRIGGER_MAGIC) {
		return;                      /* nothing staged; leave the last code */
	}
	ota_apply_note(N6_OTA_APPLY_STARTED);
	/* Disarm before any work, so a copy that dies half way is not retried at
	 * the next boot. */
	ota_bkp_write(N6_OTA_TRIGGER_BKPR, 0u);

	/* Index 1 is the PSRAM, on XSPI2; index 0 is the NOR, on XSPI1. The PSRAM
	 * is unmapped at this point, and mapping it leaves the NOR alone. */
	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(1, EXTMEM_ENABLE)) {
		ota_apply_note(N6_OTA_APPLY_ERR_PSRAM_MAP);
		ota_apply_done();
		return;
	}

	const n6_ota_psram_hdr_t *hdr = (const n6_ota_psram_hdr_t *)N6_OTA_PSRAM_BASE;
	if (hdr->magic != N6_OTA_PSRAM_HDR_MAGIC) {
		ota_apply_note(N6_OTA_APPLY_ERR_HDR_MAGIC);
		ota_apply_done();
		return;
	}
	if (hdr->hdr_crc != n6_ota_crc32(0u, (const uint8_t *)hdr, 16u)) {
		ota_apply_note(N6_OTA_APPLY_ERR_HDR_CRC);
		ota_apply_done();
		return;
	}

	const uint32_t size = hdr->size;
	if (size == 0u || size > N6_OTA_MAX_LOADABLE) {
		ota_apply_note(N6_OTA_APPLY_ERR_SIZE);
		ota_apply_done();
		return;
	}

	const uint8_t *img =
		(const uint8_t *)(N6_OTA_PSRAM_BASE + N6_OTA_PSRAM_IMAGE_OFF);
	if (n6_ota_crc32(0u, img, size) != hdr->crc32) {
		ota_apply_note(N6_OTA_APPLY_ERR_IMAGE_CRC);
		ota_apply_done();
		return;                      /* torn or decayed: leave flash alone */
	}

	/* Write the inactive slot, so a bad image can be reverted. */
	n6_ota_cb_t cb;
	int from = ota_cb_load(&cb);
	if (from < 0) {
		/* No control block, so slot A is running: record it as confirmed over
		 * the loader's whole window, so an unconfirmed update falls back to it. */
		uint32_t crc_a = 0u;
		memset(&cb, 0, sizeof(cb));
		cb.active  = 0u;             /* no control block => slot A is running */
		cb.pending = N6_OTA_NO_PENDING;
		if (flash_crc32_region(N6_OTA_SLOT_A_OFF, N6_OTA_MAX_LOADABLE, &crc_a) == 0) {
			cb.slot[0].crc32 = crc_a;
			cb.slot[0].size  = N6_OTA_MAX_LOADABLE;
			cb.slot[0].state = N6_OTA_SLOT_CONFIRMED;
		}
	}
	const uint8_t target = (uint8_t)(((cb.active <= 1u) ? cb.active : 0u) ^ 1u);
	const uint32_t base  = n6_ota_slot_off(target);

	/* Invalidate the target's record first: an interrupted copy must leave an
	 * EMPTY slot, never a stale VALID one pointing at mixed bytes. */
	cb.slot[target].state   = N6_OTA_SLOT_EMPTY;
	cb.slot[target].crc32   = 0u;
	cb.slot[target].size    = 0u;
	cb.pending              = N6_OTA_NO_PENDING;
	ota_cb_store(&cb, from);
	from = ota_cb_load(&cb);

	/* Erase only what the image needs, rounded up to whole 64 KB blocks. */
	const uint32_t span = (size + BLOCK_64K_SIZE - 1u) & ~(BLOCK_64K_SIZE - 1u);
	for (uint32_t off = 0u; off < span; off += BLOCK_64K_SIZE) {
		ota_erase_64k(base + off);
	}

	/* Program each page from RAM: feeding a page program straight from mapped
	 * PSRAM puts XSPI2 reads inside an XSPI1 write and corrupts the data. */
	static uint8_t ota_page[FLASH_PAGE_SIZE];
	for (uint32_t off = 0u; off < size; off += FLASH_PAGE_SIZE) {
		uint32_t n = size - off;
		if (n > FLASH_PAGE_SIZE) n = FLASH_PAGE_SIZE;
		memcpy(ota_page, &img[off], n);
		XSPI_WritePage(base + off, ota_page, n);
	}

	/* Verify what landed in flash. */
	uint32_t crc = 0u;
	const int vrc = flash_crc32_region(base, size, &crc);
	if (vrc != 0 || crc != hdr->crc32) {
		ota_apply_note(N6_OTA_APPLY_ERR_FLASH_CRC);
		ota_apply_done();
		return;                      /* target stays EMPTY; the active slot boots */
	}

	cb.slot[target].crc32   = crc;
	cb.slot[target].size    = size;
	cb.slot[target].version = hdr->version;
	cb.slot[target].state   = N6_OTA_SLOT_VALID;
	cb.pending              = target;
	cb.tries_left           = N6_OTA_DEFAULT_TRIES;
	ota_cb_store(&cb, from);

	ota_apply_note(N6_OTA_APPLY_OK);
	ota_apply_done();
}

/* Defined with Error_Handler. */
extern volatile int g_fsbl_soft_fail;
extern volatile int g_fsbl_soft_failed;

/* ---- OTA: program a staged asset ----------------------------------------
 * One 4 KB sector, two copies: the write goes to the copy not holding the
 * highest valid version, so an interrupted program leaves the other readable. */
static void ota_asset_note(uint32_t code)
{
	ota_bkp_write(N6_ASSET_RESULT_BKPR, code);
}

static void ota_apply_asset_from_psram(void)
{
	if (ota_bkp_read(N6_ASSET_TRIGGER_BKPR) != N6_ASSET_TRIGGER_MAGIC) {
		return;
	}
	ota_bkp_write(N6_ASSET_TRIGGER_BKPR, 0u);   /* disarm before any work */

	if (EXTMEM_OK != EXTMEM_MemoryMappedMode(1, EXTMEM_ENABLE)) {
		ota_asset_note(N6_ASSET_RESULT_ERR_HDR);
		return;
	}

	const n6_asset_hdr_t *h = (const n6_asset_hdr_t *)N6_ASSET_PSRAM_BASE;
	if (h->magic != N6_ASSET_PSRAM_MAGIC ||
	    h->hdr_crc != n6_ota_crc32(0u, (const uint8_t *)h, 16u)) {
		ota_asset_note(N6_ASSET_RESULT_ERR_HDR);
		ota_apply_done();
		return;
	}
	if (h->len == 0u || h->len > N6_ASSET_MAX_BYTES) {
		ota_asset_note(N6_ASSET_RESULT_ERR_SIZE);
		ota_apply_done();
		return;
	}

	const uint8_t *src = (const uint8_t *)(N6_ASSET_PSRAM_BASE + N6_ASSET_HDR_BYTES);
	if (n6_ota_crc32(0u, src, h->len) != h->crc32) {
		ota_asset_note(N6_ASSET_RESULT_ERR_CRC);
		ota_apply_done();
		return;
	}

	/* Target: the copy that is not the best valid one. */
	uint32_t best_ver = 0u; int best = -1;
	for (uint32_t c = 0u; c < N6_ASSET_COPIES; c++) {
		n6_asset_hdr_t e;
		if (flash_read_region(N6_ASSET_OFF(c), (uint8_t *)&e, sizeof(e)) != 0) continue;
		if (e.magic != N6_ASSET_MAGIC) continue;
		if (e.hdr_crc != n6_ota_crc32(0u, (const uint8_t *)&e, 16u)) continue;
		if (best < 0 || e.version > best_ver) { best = (int)c; best_ver = e.version; }
	}
	const uint32_t target = (best == 0) ? 1u : 0u;
	const uint32_t off    = N6_ASSET_OFF(target);

	/* Build the sector in RAM, the header with the on-flash magic and then the
	 * payload, and program from there, not from mapped PSRAM. */
	static uint8_t sector[N6_ASSET_SECTOR];
	memset(sector, 0xFF, sizeof(sector));
	n6_asset_hdr_t *out = (n6_asset_hdr_t *)sector;
	out->magic   = N6_ASSET_MAGIC;
	out->version = h->version;
	out->len     = h->len;
	out->crc32   = h->crc32;
	out->hdr_crc = n6_ota_crc32(0u, (const uint8_t *)out, 16u);
	memcpy(sector + N6_ASSET_HDR_BYTES, src, h->len);

	/* From here to the end of the write, a HAL failure must unwind rather than
	 * hang: see g_fsbl_soft_fail on Error_Handler. */
	g_fsbl_soft_fail = 1;
	g_fsbl_soft_failed = 0;
	ota_erase_4k(off);
	for (uint32_t i = 0u; i < N6_ASSET_HDR_BYTES + h->len; i += FLASH_PAGE_SIZE) {
		uint32_t n = (N6_ASSET_HDR_BYTES + h->len) - i;
		if (n > FLASH_PAGE_SIZE) n = FLASH_PAGE_SIZE;
		XSPI_WritePage(off + i, &sector[i], n);
	}
	if (g_fsbl_soft_failed) {
		g_fsbl_soft_fail = 0;
		ota_asset_note(N6_ASSET_RESULT_ERR_VERIFY);
		ota_apply_done();
		return;
	}

	/* Verify what landed in flash. */
	n6_asset_hdr_t chk;
	uint8_t payload_crc_ok = 0u;
	if (flash_read_region(off, (uint8_t *)&chk, sizeof(chk)) == 0 &&
	    chk.magic == N6_ASSET_MAGIC &&
	    chk.hdr_crc == n6_ota_crc32(0u, (const uint8_t *)&chk, 16u) &&
	    chk.len == h->len && chk.crc32 == h->crc32) {
		uint32_t got = 0u;
		if (flash_crc32_region(off + N6_ASSET_HDR_BYTES, h->len, &got) == 0 &&
		    got == h->crc32) {
			payload_crc_ok = 1u;
		}
	}
	g_fsbl_soft_fail = 0;
	ota_asset_note(payload_crc_ok ? N6_ASSET_RESULT_OK : N6_ASSET_RESULT_ERR_VERIFY);
	ota_apply_done();
}

/* Boot the chosen slot, or when nothing boots, set FORCEDFU and reset into
 * DFU. Does not return. */
static void ota_boot(void)
{
	/* Commit a trial that reported success, then apply what the app staged in
	 * PSRAM. Each step does nothing unless its backup-register magic is set. */
	ota_commit_if_confirmed();
	ota_apply_from_psram();
	ota_apply_asset_from_psram();

	n6_ota_cb_t cb;
	int from = ota_cb_load(&cb);

	if (from >= 0) {
		uint8_t order[2];
		int     n = 0;

		if (cb.pending != N6_OTA_NO_PENDING && cb.pending <= 1u && cb.tries_left > 0u) {
			/* A pending image's try is counted before the jump, so an image
			 * that boots and hangs still uses one. */
			cb.tries_left -= 1u;
			ota_cb_store(&cb, from);
			from = (from == 0) ? 1 : 0;   /* store target alternated */

			order[n++] = cb.pending;
			if (cb.active != cb.pending) order[n++] = cb.active;
		} else {
			if (cb.pending != N6_OTA_NO_PENDING) {
				/* Tries exhausted: the update failed. Cancel it; the app sees
				 * the revert in the slot states. */
				cb.pending = N6_OTA_NO_PENDING;
				ota_cb_store(&cb, from);
				from = (from == 0) ? 1 : 0;
			}
			order[n++] = cb.active;
			if ((cb.active ^ 1u) <= 1u) order[n++] = cb.active ^ 1u;
		}

		for (int i = 0; i < n; i++) {
			_Bool ok = ota_slot_ok(&cb, order[i]);
			if (ok) {
				BOOT_SetSourceOffset(n6_ota_slot_off(order[i]));
				(void)BOOT_Application();
				/* returns only on load failure; fall through */
			}
			/* Failed (bad CRC, or the loader returned). A failed pending slot is
			 * cancelled, or the other image's n6_ota_confirm() would activate it. */
			if (cb.pending != N6_OTA_NO_PENDING && order[i] == cb.pending) {
				cb.pending = N6_OTA_NO_PENDING;
				ota_cb_store(&cb, from);
				from = (from == 0) ? 1 : 0;
			}
		}
		/* fall through: nothing CB-tracked would boot */
	}

	/* No control block, or no tracked slot booted: slot A. */
	BOOT_SetSourceOffset(N6_OTA_SLOT_A_OFF);
	(void)BOOT_Application();

	/* Nothing boots: force DFU on the next boot; backup registers survive a
	 * software reset. */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, FORCEDFU);
	HAL_NVIC_SystemReset();
}

/* NOR at 200 MHz for the application, in this order: mapping off, prescaler 0,
 * a calibrating read, reads at the DQS delay, the mapping commands, the delay
 * again, then the mapping. On failure the NOR is mapped at prescaler 1. */

#define FAST_DQS_FINE    16u
#define FAST_DQS_COARSE   3u
#define FAST_WARM_LEN   256u         /* enough of a transfer for the DLL */
#define FAST_READ_LEN   0x10000u     /* read at the delay before mapping */
#define FAST_CHUNK      4096u
#define FAST_XFER_MS    2000u        /* a stalled transfer costs this once */

static uint8_t fast_buf[FAST_CHUNK];

static HAL_StatusTypeDef fast_set_delay(uint32_t type, uint32_t fine, uint32_t coarse)
{
	XSPI_HSCalTypeDef cal;

	cal.DelayValueType        = type;
	cal.FineCalibrationUnit   = fine;
	cal.CoarseCalibrationUnit = coarse;
	cal.MaxCalibration        = 0u;
	return HAL_XSPI_SetDelayValue(&hxspi1, &cal);
}

/* The read EXTMEM_MemoryMappedMode maps: octal DTR, 19 dummy cycles, sampled
 * on DQS. */
static void fast_read_cmd(XSPI_RegularCmdTypeDef *cmd, uint32_t optype)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->OperationType      = optype;
	cmd->Instruction        = OCTAL_IO_DTR_READ_CMD;
	cmd->InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
	cmd->InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
	cmd->InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	cmd->AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
	cmd->AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
	cmd->AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
	cmd->AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	cmd->DataMode           = HAL_XSPI_DATA_8_LINES;
	cmd->DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
	cmd->DummyCycles        = 19u;
	cmd->DQSMode            = HAL_XSPI_DQS_ENABLE;
	cmd->IOSelect           = HAL_XSPI_SELECT_IO_7_0;
}

/* Read `n` bytes at flash offset `off` into fast_buf, NOR unmapped. A nonzero
 * `delay` sets the DQS delay after the command, which arms a DLL calibration;
 * returns 0, or 1 command, 2 delay, 3 transfer failed. */
static int fast_read(uint32_t off, uint32_t n, int delay)
{
	XSPI_RegularCmdTypeDef cmd;

	fast_read_cmd(&cmd, HAL_XSPI_OPTYPE_COMMON_CFG);
	cmd.Address    = off;
	cmd.DataLength = n;
	if (HAL_XSPI_Command(&hxspi1, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		(void)HAL_XSPI_Abort(&hxspi1);
		return 1;
	}
	if ((delay != 0) && (fast_set_delay(HAL_XSPI_CAL_DQS_INPUT_DELAY,
	                                    FAST_DQS_FINE, FAST_DQS_COARSE) != HAL_OK)) {
		(void)HAL_XSPI_Abort(&hxspi1);
		return 2;
	}
	if (HAL_XSPI_Receive(&hxspi1, fast_buf, FAST_XFER_MS) != HAL_OK) {
		(void)HAL_XSPI_Abort(&hxspi1);
		return 3;
	}
	return 0;
}

/* EXTMEM_MemoryMappedMode(0, EXTMEM_ENABLE)'s sequence, NOR unmapped, with the
 * DQS delay set between the commands and the mapping. Returns 0, or 1 command,
 * 2 delay, 3 mapping failed. */
static int fast_map(void)
{
	XSPI_RegularCmdTypeDef cmd;
	XSPI_MemoryMappedTypeDef map;

	fast_read_cmd(&cmd, HAL_XSPI_OPTYPE_READ_CFG);
	if (HAL_XSPI_Command(&hxspi1, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return 1;
	}
	cmd.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
	cmd.Instruction   = OCTAL_PAGE_PROG_CMD;
	cmd.DummyCycles   = 0u;
	if (HAL_XSPI_Command(&hxspi1, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return 1;
	}
	if (fast_set_delay(HAL_XSPI_CAL_DQS_INPUT_DELAY,
	                   FAST_DQS_FINE, FAST_DQS_COARSE) != HAL_OK) {
		return 2;
	}
	memset(&map, 0, sizeof(map));
	map.NoPrefetchAXI     = HAL_XSPI_AXI_PREFETCH_ENABLE;
	map.NoPrefetchData    = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
	map.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
	if (HAL_XSPI_MemoryMapped(&hxspi1, &map) != HAL_OK) {
		return 3;
	}
	return 0;
}

void FSBL_XSPI1_Fast(uint32_t src_off)
{
	/* The DLL's delays for 100 MHz, restored if the switch to 200 MHz is
	 * abandoned after they change. */
	const uint32_t sir_100 = READ_REG(hxspi1.Instance->CALSIR);
	const uint32_t sor_100 = READ_REG(hxspi1.Instance->CALSOR);
	int faster = 0;                       /* prescaler set to 0 */
	int fast   = 0;                       /* NOR mapped at 200 MHz */

	if ((EXTMEM_OK == EXTMEM_MemoryMappedMode(0, EXTMEM_DISABLE)) &&
	    (HAL_XSPI_SetClockPrescaler(&hxspi1, 0u) == HAL_OK)) {
		faster = 1;
		/* The DLL calibrates in the first read; the rest run at the delay. */
		fast = (fast_read(src_off, FAST_WARM_LEN, 0) == 0);
		for (uint32_t off = 0u; (fast != 0) && (off < FAST_READ_LEN); off += FAST_CHUNK) {
			fast = (fast_read(src_off + off, FAST_CHUNK, 1) == 0);
		}
		fast = (fast != 0) && (fast_map() == 0);
	}

	if (fast == 0) {
		if (faster != 0) {
			/* Back to 100 MHz: unmapped, then a transfer to recalibrate the
			 * DLL. */
			(void)EXTMEM_MemoryMappedMode(0, EXTMEM_DISABLE);
			(void)HAL_XSPI_SetClockPrescaler(&hxspi1, 1u);
			if (fast_read(src_off, FAST_WARM_LEN, 0) != 0) {
				/* No transfer to calibrate with: put back what it held. */
				(void)fast_set_delay(HAL_XSPI_CAL_DQS_INPUT_DELAY,
				                     sir_100 & 0x7Fu, (sir_100 >> 16) & 0x1Fu);
				(void)fast_set_delay(HAL_XSPI_CAL_DATA_OUTPUT_DELAY,
				                     sor_100 & 0x7Fu, (sor_100 >> 16) & 0x1Fu);
			}
		}
		/* Mapped again, or still mapped, which this refuses harmlessly. */
		(void)EXTMEM_MemoryMappedMode(0, EXTMEM_ENABLE);
	}
}

static inline void RCC_Set_PeriphClockSelectors_ToTableValues(void)
{
    /* RCC_CCIPR6:
       XSPI1SEL[1:0]        = 0x0
       OTGPHY1CKREFSEL      = 0x0
    */
    MODIFY_REG(RCC->CCIPR6,
               RCC_CCIPR6_XSPI1SEL_Msk | RCC_CCIPR6_OTGPHY1CKREFSEL_Msk,
               (0u << RCC_CCIPR6_XSPI1SEL_Pos) |
               (0u << RCC_CCIPR6_OTGPHY1CKREFSEL_Pos));

    /* RCC_CCIPR7:
       PERSEL[2:0]          = 0x0
       RTCSEL[1:0]          = 0x2
    */
    MODIFY_REG(RCC->CCIPR7,
               RCC_CCIPR7_PERSEL_Msk | RCC_CCIPR7_RTCSEL_Msk,
               (0u << RCC_CCIPR7_PERSEL_Pos) |
               (2u << RCC_CCIPR7_RTCSEL_Pos));

    /* RCC_CCIPR8:
       SDMMC1SEL[1:0]       = 0x0
       SDMMC2SEL[1:0]       = 0x0
    */
    MODIFY_REG(RCC->CCIPR8,
               RCC_CCIPR8_SDMMC1SEL_Msk | RCC_CCIPR8_SDMMC2SEL_Msk,
               (0u << RCC_CCIPR8_SDMMC1SEL_Pos) |
               (0u << RCC_CCIPR8_SDMMC2SEL_Pos));

    /* RCC_CCIPR13:
       USART1SEL[2:0]       = 0x0
       USART2SEL[2:0]       = 0x0
       UART4SEL[2:0]        = 0x0
       UART5SEL[2:0]        = 0x0
    */
    MODIFY_REG(RCC->CCIPR13,
               RCC_CCIPR13_USART1SEL_Msk |
               RCC_CCIPR13_USART2SEL_Msk |
               RCC_CCIPR13_UART4SEL_Msk  |
               RCC_CCIPR13_UART5SEL_Msk,
               (0u << RCC_CCIPR13_USART1SEL_Pos) |
               (0u << RCC_CCIPR13_USART2SEL_Pos) |
               (0u << RCC_CCIPR13_UART4SEL_Pos)  |
               (0u << RCC_CCIPR13_UART5SEL_Pos));
}
volatile uint32_t enterdfu = 0;

int main(void)
{
  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();


  HAL_PWR_EnableBkUpAccess();          // Required for RTC backup registers.


  MX_TIM1_Init();                      // TIM1 used for PWM pulsing Red LED.

  FLASH_BB_Init();                     // Default state of flash is SPI, configure OSPI
  FLASH_BB_EnterDTR_OPI();             //   through bit-banging single-lane SPI.

  MX_XSPI1_Init();
  MX_XSPI2_Init();
  MX_EXTMEM_MANAGER_Init();

  RCC_Set_PeriphClockSelectors_ToTableValues();


  if (PSRAM_GlobalReset(&hxspi2) != 0)
  {
      Error_Handler();
  }

  /* MR0 = 0x30 : Fixed latency, LC=7, full drive */
  if (PSRAM_WriteCR(&hxspi2, 0x30u, 0x00000000u) != 0)
  {
      Error_Handler();
  }

  /* MR4 = 0x20 : WLC=7, default refresh/PASR bits left at 0 */
  if (PSRAM_WriteCR(&hxspi2, 0x20u, 0x00000004u) != 0)
  {
      Error_Handler();
  }

  if (PSRAM_WriteCR(&hxspi2, 0x0Bu, 0x00000008u) != 0)
  {
      Error_Handler();
  }
  /* MR8 = 0x40 is not valid for the x8 APS51208N. */

  HAL_XSPI_SetClockPrescaler(&hxspi2, 0);


  MX_RTC_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1024);  // Red LED off.

  /* Double-tap timing from the RTC, at 1 s resolution. */
  RTC_TimeTypeDef t;
  RTC_DateTypeDef d;
  HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

  uint32_t now_ms  = (t.Hours * 3600000UL) + (t.Minutes * 60000UL) + (t.Seconds * 1000UL);
  uint32_t last_ms = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  uint32_t magic   = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

  const uint32_t window = 900;

  enterdfu = 0;

  if (magic == MAGIC)
  {
    if ((now_ms - last_ms) < window)
    {
    	enterdfu = 1;
    }
  }
  if (magic == FORCEDFU)
  {
	  enterdfu = 1;
  }

  if (enterdfu == 1){
      /* -------- Enter DFU mode -------- */
      MX_USB1_OTG_HS_PCD_Init();
      tusb_init();
      g_usb_up = true;

      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, 0);  // Clear the entry key after DFU mode is selected.

      /* DFU state */
      dfu_start = false;
      dfu_data_mode = false;
      app_base_addr = APP_XSPI_BASE_DEFAULT;
      app_write_addr = APP_XSPI_BASE_DEFAULT;
      page_fill = 0;
      bytes_since_ack = 0;
      rx_len = 0;

      /* LED breathing */
      uint16_t duty = 0;
      int dir = 1;
      uint32_t last_led_tick = HAL_GetTick();

      while (1)
      {
        tud_task();
        cdc_task();

        /* Process every complete buffered frame */
        bool did_work = false;
        while (try_get_one_frame())
        {
          did_work = true;

          uint16_t datalen = frame_payload_len();
          uint8_t *data = frame_payload_ptr();

          /* ---- v2 commands, valid outside the data stream ---- */

          /* DFU_INFO: capability probe. Unknown commands are ACKed too, so a
           * host detects support by the N6INFO line, not by ACK or NAK. */
          if (!dfu_data_mode && datalen == 8 && memcmp(data, "DFU_INFO", 8) == 0)
          {
            uint32_t uid[3];
            n6_read_uid_words(uid);
            char msg[80];
            snprintf(msg, sizeof(msg),
                     "N6INFO 2 CRC32 MULTI OTA-AB LOAD1536K UID %08lX%08lX%08lX\r\n",
                     (unsigned long)uid[0], (unsigned long)uid[1],
                     (unsigned long)uid[2]);
            usb_log(msg);
            usb_log("N6ACK\r\n");
            continue;
          }

          /* DFU_CRC32 <u32 offset> <u32 len>: CRC-32 of flash contents. */
          if (!dfu_data_mode && datalen == (9 + 8) && memcmp(data, "DFU_CRC32", 9) == 0)
          {
            uint32_t offset, len;
            memcpy(&offset, &data[9], sizeof(uint32_t));
            memcpy(&len, &data[13], sizeof(uint32_t));
            uint32_t crc;
            if (len == 0 || offset >= FLASH_SIZE || len > (FLASH_SIZE - offset))
            {
              usb_log("CRC_BAD_ARGS\r\n");
            }
            else if (flash_crc32_region(offset, len, &crc) != 0)
            {
              usb_log("CRC_READ_FAIL\r\n");
            }
            else
            {
              char msg[32];
              snprintf(msg, sizeof(msg), "CRC32 0x%08lX\r\n", (unsigned long)crc);
              usb_log(msg);
            }
            usb_log("N6ACK\r\n");
            continue;
          }

          /* DFU_RESET: reboot into the application from idle state (used
           * after a DFU_NEXT-terminated final image). */
          if (!dfu_data_mode && datalen == 9 && memcmp(data, "DFU_RESET", 9) == 0)
          {
            usb_log("RESETTING\r\n");
            tud_task();
            HAL_Delay(50);   /* let the reply drain to the host */
            RCC_Set_PeriphClockSelectors_ToTableValues();
            HAL_NVIC_SystemReset();
          }

          /* ---- DFU START (with address) ---- */
          /* Format: "DFU START" (9 bytes) + uint32 LE address (4 bytes) = 13 bytes */
          if (datalen == 13 && memcmp(data, "DFU START", 9) == 0)
          {
            uint32_t addr;
            memcpy(&addr, &data[9], sizeof(uint32_t));

            /* Validate address: must be sector-aligned and within flash bounds */
            if ((addr % XSPI_SECTOR_SIZE) != 0 || addr >= FLASH_SIZE)
            {
              usb_log("START_BAD_ADDR\r\n");
              continue;
            }

            app_base_addr = addr;
            app_write_addr = addr;

            dfu_start = true;
            dfu_data_mode = false;
            page_fill = 0;
            bytes_since_ack = 0;

            char msg[48];
            snprintf(msg, sizeof(msg), "ADDR_SET 0x%08lX\r\n", (unsigned long)addr);
            usb_log(msg);
            usb_log("N6ACK\r\n");
            continue;
          }

          /* ---- DFU ERASE ---- */
          if (dfu_start && datalen == (9 + 4) && memcmp(data, "DFU_ERASE", 9) == 0)
          {
            uint32_t erase_size;
            memcpy(&erase_size, &data[9], sizeof(uint32_t));

            /* Non-zero, sector-aligned and inside the base's region (edges from
             * n6_ota_layout.h). A bad size is reported and DFU continues. */
            uint32_t region_end = FLASH_SIZE;
            if      (app_base_addr <  N6_OTA_SLOT_A_OFF)              region_end = N6_OTA_SLOT_A_OFF;
            else if (app_base_addr <  N6_OTA_CB_OFF(0))               region_end = N6_OTA_CB_OFF(0);
            else if (app_base_addr <  0x00300000u)                    region_end = 0x00300000u;   /* CB + reserved */
            else if (app_base_addr <  N6_OTA_SLOT_B_OFF)              region_end = N6_OTA_SLOT_B_OFF; /* weights pool */
            else if (app_base_addr <  N6_OTA_SLOT_B_OFF + N6_OTA_SLOT_SIZE) region_end = N6_OTA_SLOT_B_OFF + N6_OTA_SLOT_SIZE;

            if (erase_size == 0 ||
                (erase_size % XSPI_SECTOR_SIZE) != 0 ||
                (app_base_addr + erase_size) > region_end)
            {
              usb_log("ERASE_BAD_SIZE\r\n");
              dfu_start = false;
              continue;
            }

            app_erase_size = erase_size;

            /* Ensure flash is configured for Octal DTR ops */
            XSPI_NOR_OctalDTRModeCfg(&hxspi1);
            HAL_XSPI_Abort(&hxspi1);

            XSPI_EraseAppArea(&hxspi1);

            /* Erase completed */
            usb_log("N6ACK\r\n");
            continue;
          }

          /* ---- DFU DATA (enter write stage) ---- */
          if (dfu_start && datalen == 8 && memcmp(data, "DFU_DATA", 8) == 0)
          {
            usb_log("N6ACK\r\n");

            dfu_start = false;
            dfu_data_mode = true;

            app_write_addr = app_base_addr;  /* Use address from DFU START */
            page_fill = 0;
            bytes_since_ack = 0;
            continue;
          }

          /* ---- DATA MODE ---- */
          if (dfu_data_mode)
          {
            if (datalen == 7 && memcmp(data, "DFU_END", 7) == 0)
            {
              /* Flush partial page */
              if (page_fill > 0)
              {
                XSPI_WritePage(app_write_addr, page_buf, page_fill);
                app_write_addr += page_fill;
                page_fill = 0;
              }

              dfu_data_mode = false;

              /* Note an upload to an OTA slot before the reset below, or the
               * slot arbitration does not see it. */
              ota_dfu_note_image(app_base_addr, app_write_addr - app_base_addr);

              usb_log("DFU_DONE\r\n");

              RCC_Set_PeriphClockSelectors_ToTableValues();
              HAL_NVIC_SystemReset();
            }

            /* DFU_NEXT (v2): finish this image and return to idle, keeping the
             * session open for more images and a final DFU_RESET. */
            if (datalen == 8 && memcmp(data, "DFU_NEXT", 8) == 0)
            {
              if (page_fill > 0)
              {
                XSPI_WritePage(app_write_addr, page_buf, page_fill);
                app_write_addr += page_fill;
                page_fill = 0;
              }
              dfu_data_mode = false;
              dfu_start = false;
              /* Note an upload to an OTA slot, as DFU_END does. */
              ota_dfu_note_image(app_base_addr, app_write_addr - app_base_addr);
              usb_log("NEXT_OK\r\n");
              usb_log("N6ACK\r\n");
              continue;
            }

            /* Any other frame is firmware data: the two commands above match by
             * full length and content. */
            dfu_feed_data(data, datalen);

            /* The host tools expect an ACK for every frame. */
            usb_log("N6ACK\r\n");
            continue;
          }

          /* Unknown command outside data mode: ACK it so the host does not
             stall. */
          usb_log("N6ACK\r\n");
        }

        /* LED breathing at ~1ms granularity without blocking USB */
        uint32_t now = HAL_GetTick();
        if ((now - last_led_tick) >= 1)
        {
          last_led_tick = now;

          duty = (uint16_t)(duty + (dir * 10));
          if (duty >= 1000) { duty = 1000; dir = -1; }
          if (duty == 0)    { dir =  1; }

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
        }

        /* Nothing processed: wait 1 ms */
        if (!did_work)
        {
          HAL_Delay(1);
        }
      }
  }
  else
  {
    /* Arm the double-tap window, then boot; ota_boot() does not return. */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, now_ms);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, MAGIC);

    ota_boot();
  }

  /* Should never reach here */
  while (1) { }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
  Error_Handler();
  }
  /* Enable HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_Delay(HSE_STARTUP_TIMEOUT);

  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;

  /* PLL1 = 64 x 25 / 2 = 800MHz */
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 2;
  RCC_OscInitStruct.PLL1.PLLN = 25;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;

  /* PLL2 = 64 x 125 / 8 = 1000MHz */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL2.PLLM = 8;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLN = 125;
  RCC_OscInitStruct.PLL2.PLLP1 = 1;
  RCC_OscInitStruct.PLL2.PLLP2 = 1;

  /* PLL3 = (64 x 225 / 8) / (1 * 2) = 900MHz */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 8;
  RCC_OscInitStruct.PLL3.PLLN = 225;
  RCC_OscInitStruct.PLL3.PLLFractional = 0;
  RCC_OscInitStruct.PLL3.PLLP1 = 1;
  RCC_OscInitStruct.PLL3.PLLP2 = 2;

  /* PLL4 = (48 x 200 / 6) / (4 * 1) = 400 MHz */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 6;
  RCC_OscInitStruct.PLL4.PLLN = 200;
  RCC_OscInitStruct.PLL4.PLLFractional = 0;
  RCC_OscInitStruct.PLL4.PLLP1 = 4;
  RCC_OscInitStruct.PLL4.PLLP2 = 1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                              |RCC_CLOCKTYPE_PCLK4;


  /* CPU CLock (sysa_ck) = ic1_ck = PLL1 output/ic1_divider = 800 MHz */
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 1;

  /* AXI Clock (sysb_ck) = ic2_ck = PLL4 output/ic2_divider = 400 MHz */
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL4;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 1;

  /* NPU Clock (sysc_ck) = ic6_ck = PLL2 output/ic6_divider = 1000 MHz */
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL2;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 1;

  /* AXISRAM3/4/5/6 Clock (sysd_ck) = ic11_ck = PLL3 output/ic11_divider = 900 MHz */
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 1;

  /* HCLK = sysb_ck / HCLK divider = 200 MHz */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

  /* PCLKx = HCLK / PCLKx divider = 200 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }




}

/**
 * @brief XSPI1 Initialization Function
 * @param None
 * @retval None
 */

static void MX_XSPI1_Init(void)
{

  /* USER CODE BEGIN XSPI1_Init 0 */

  /* USER CODE END XSPI1_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI1_Init 1 */

  /* USER CODE END XSPI1_Init 1 */
  /* XSPI1 parameter configuration*/
  hxspi1.Instance = XSPI1;
  hxspi1.Init.FifoThresholdByte = 8;
  hxspi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi1.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_256MB;
  hxspi1.Init.ChipSelectHighTimeCycle = 8;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi1.Init.ClockPrescaler = 1;
  hxspi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi1.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  hxspi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi1.Init.MaxTran = 100;
  hxspi1.Init.Refresh = 0;
  hxspi1.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi1) != HAL_OK)
  {
    Error_Handler();
  }

  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 10;
  if (HAL_XSPIM_Config(&hxspi1, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI1_Init 2 */

  /* USER CODE END XSPI1_Init 2 */

}
/**
 * @brief XSPI2 Initialization Function
 * @param None
 * @retval None
 */


static void MX_XSPI2_Init(void)
{

  /* USER CODE BEGIN XSPI2_Init 0 */

  /* USER CODE END XSPI2_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI2_Init 1 */

  /* USER CODE END XSPI2_Init 1 */
  /* XSPI2 parameter configuration*/
  hxspi2.Instance = XSPI2;
  hxspi2.Init.FifoThresholdByte = 4;
  hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_APMEM;
  hxspi2.Init.MemorySize = HAL_XSPI_SIZE_256MB;
  hxspi2.Init.ChipSelectHighTimeCycle = 5;
  hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi2.Init.ClockPrescaler = 3;
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_16KB;
  hxspi2.Init.MaxTran = 100;
  /* Release chip select periodically so the PSRAM can refresh: it cannot while
   * CS is low, and a burst past tCEM (about 4 us) loses data. */
  hxspi2.Init.Refresh = 400;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS2;
  if (HAL_XSPI_Init(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }

  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 10;
  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI2_Init 2 */

  /* USER CODE END XSPI2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPION_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();

	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = BLUE_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = RED_LED_Pin;            // PE11
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;    // TIM1_CH2
	HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @brief  Bit-banged SPI pins to the NOR, used before the XSPI takes over.
 */
static void FLASH_BB_Init(void) {
	/* Port N clock, also enabled by MX_GPIO_Init */
	__HAL_RCC_GPION_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* CS, CLK, MOSI as outputs */
	GPIO_InitStruct.Pin = FLASH_CS_PIN | FLASH_CLK_PIN | FLASH_MOSI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(FLASH_CS_GPIO, &GPIO_InitStruct);

	/* MISO as input */
	GPIO_InitStruct.Pin = FLASH_MISO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(FLASH_MISO_GPIO, &GPIO_InitStruct);

	/* Idle levels: CS high, CLK low, MOSI low */
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FLASH_CLK_GPIO, FLASH_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FLASH_MOSI_GPIO, FLASH_MOSI_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Short busy-wait between bit-banged SPI edges. Its length depends on
 *         the CPU clock.
 */
static void FLASH_BB_Delay(void) {
	for (volatile int i = 0; i < 100; i++) {
		__NOP();
	}
}

/**
 * @brief  Send one byte over bit-banged SPI, mode 0 (CPOL = 0, CPHA = 0), MSB
 *         first.
 */
static void FLASH_BB_SendByte(uint8_t out) {
	for (int i = 7; i >= 0; i--) {
		/* Set MOSI on falling edge */
		HAL_GPIO_WritePin(FLASH_MOSI_GPIO, FLASH_MOSI_PIN,
				(out & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		FLASH_BB_Delay();

		/* Rising edge: slave samples data */
		HAL_GPIO_WritePin(FLASH_CLK_GPIO, FLASH_CLK_PIN, GPIO_PIN_SET);
		FLASH_BB_Delay();

		/* Falling edge */
		HAL_GPIO_WritePin(FLASH_CLK_GPIO, FLASH_CLK_PIN, GPIO_PIN_RESET);
		FLASH_BB_Delay();
	}
}

/**
 * @brief  Read one byte over bit-banged SPI, mode 0, MSB first.
 */
static uint8_t FLASH_BB_ReadByte(void) {
	uint8_t in = 0;

	for (int i = 7; i >= 0; i--) {
		FLASH_BB_Delay();

		/* Rising edge: sample MISO */
		HAL_GPIO_WritePin(FLASH_CLK_GPIO, FLASH_CLK_PIN, GPIO_PIN_SET);
		FLASH_BB_Delay();

		if (HAL_GPIO_ReadPin(FLASH_MISO_GPIO, FLASH_MISO_PIN) == GPIO_PIN_SET) {
			in |= (1U << i);
		}

		/* Falling edge */
		HAL_GPIO_WritePin(FLASH_CLK_GPIO, FLASH_CLK_PIN, GPIO_PIN_RESET);
	}

	return in;
}

/**
 * @brief  Read the NOR status register (RDSR) over bit-banged SPI.
 */

static uint8_t FLASH_BB_ReadStatus(void) {
	uint8_t sr;

	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_BB_SendByte(FLASH_CMD_RDSR);
	sr = FLASH_BB_ReadByte();
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);

	return sr;
}

/**
 * @brief  Poll the NOR status register over bit-banged SPI until WIP clears.
 */

static void FLASH_BB_WaitWhileBusy(void) {
	while (FLASH_BB_ReadStatus() & 0x01)   // bit0 = WIP
	{
		FLASH_BB_Delay();
	}
}

/**
 * @brief  Switch the NOR from single-line SPI to octal DTR over bit-banged SPI,
 *         before the XSPI is initialised.
 */
void FLASH_BB_EnterDTR_OPI(void) {
	/* 1. Write enable. */
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_BB_SendByte(FLASH_CMD_WREN);
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);
	FLASH_BB_Delay();

	/* 2. Write CR2 at address 0x00000000 with the DTR OPI mode value. */
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_BB_SendByte(FLASH_CMD_WRCR2);
	FLASH_BB_SendByte(FLASH_CR2_ADDR0_A1);
	FLASH_BB_SendByte(FLASH_CR2_ADDR0_A2);
	FLASH_BB_SendByte(FLASH_CR2_ADDR0_A3);
	FLASH_BB_SendByte(FLASH_CR2_ADDR0_A4);
	FLASH_BB_SendByte(FLASH_CR2_DTR_OPI);
	HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);

	/* 3. Wait for the mode change write to finish. */
	FLASH_BB_WaitWhileBusy();
}

/**
 * @brief  RTC for its backup registers; does not set the time or date. Needs
 *         backup-domain access (HAL_PWR_EnableBkUpAccess()) first.
 */

static void MX_RTC_Init(void) {

	RTC_PrivilegeStateTypeDef privilegeState = { 0 };
	RTC_SecureStateTypeDef secureState = { 0 };

	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
	hrtc.Init.BinMode = RTC_BINARY_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
	privilegeState.rtcPrivilegeFeatures = RTC_PRIVILEGE_FEATURE_INIT
			| RTC_PRIVILEGE_FEATURE_TS;
	privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
	privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
	privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
	if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK) {
		Error_Handler();
	}
	secureState.rtcSecureFull = RTC_SECURE_FULL_YES;
	secureState.backupRegisterStartZone2 = RTC_BKP_DR0;
	secureState.backupRegisterStartZone3 = RTC_BKP_DR0;
	if (HAL_RTCEx_SecureModeSet(&hrtc, &secureState) != HAL_OK) {
		Error_Handler();
	}

	HAL_RTCEx_SecureModeSet(&hrtc, &secureState);

}

/**
 * @brief  TIM1 channel 2 PWM for the red LED, with 1000 duty-cycle steps. The
 *         caller starts it with HAL_TIM_PWM_Start().
 */

void MX_TIM1_Init(void) {
	__HAL_RCC_TIM1_CLK_ENABLE();

	htim1.Instance = TIM1;

	/* Timer clock / 160. */
	htim1.Init.Prescaler = 159;

	/* 1000 brightness steps. */
	htim1.Init.Period = 999;

	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		Error_Handler();

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();
}

/**
 * @brief  Append received CDC data to rx_buf, the buffer try_get_one_frame()
 *         parses. Called from the main loop and between erase steps.
 */

static void cdc_task(void) {
    uint8_t itf;
    for (itf = 0; itf < CFG_TUD_CDC; itf++) {
        while (tud_cdc_n_available(itf)) {
            uint32_t space = RX_BUF_SIZE - rx_len;
            if (space == 0) {
                /* Drop the buffered stream if the host overruns the receive buffer. */
                rx_len = 0;
                return;
            }
            uint32_t n = tud_cdc_n_read(itf, &rx_buf[rx_len], space);
            if (n == 0) return;
            rx_len += n;
        }
    }
}

static bool try_get_one_frame(void) {
    /* Need at least STX, LEN16, CHK, and ETX. */
    if (rx_len < 6) return false;

    /* Search for the next frame start. */
    uint32_t stx = 0;
    while (stx < rx_len && rx_buf[stx] != 0x02) stx++;
    if (stx > 0) {
        memmove(rx_buf, &rx_buf[stx], rx_len - stx);
        rx_len -= stx;
        if (rx_len < 6) return false;
    }

    /* Decode the 16-bit payload length. */
    uint16_t datalen = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
    uint32_t needed = (uint32_t)datalen + 5;  // STX + LEN16 + DATA + CHK + ETX
    if (needed > RX_BUF_SIZE) {
        /* Reject malformed lengths that cannot fit in the working buffer. */
        rx_len = 0;
        return false;
    }
    if (rx_len < needed) return false;

    /* Copy the candidate frame into the dedicated work buffer. */
    memcpy(frame, rx_buf, needed);
    frame_len = needed;

    /* Remove the candidate frame from the receive queue. */
    memmove(rx_buf, &rx_buf[needed], rx_len - needed);
    rx_len -= needed;

    /* Validate framing and checksum before handing the frame to the protocol layer. */
    if (!validate_packet(frame, (int)frame_len)) {
        return false;
    }
    return true;
}
/**
 * @brief  HAL PCD set-up for USB OTG HS in device mode on the embedded PHY;
 *         tusb_init() follows.
 */
void MX_USB1_OTG_HS_PCD_Init(void) {

	hpcd_USB_OTG_HS1.Instance = USB1_OTG_HS;
	hpcd_USB_OTG_HS1.Init.dev_endpoints = 9;
	hpcd_USB_OTG_HS1.Init.speed = PCD_SPEED_HIGH;
	hpcd_USB_OTG_HS1.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
	hpcd_USB_OTG_HS1.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_HS1.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_HS1.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_HS1.Init.use_dedicated_ep1 = DISABLE;
	hpcd_USB_OTG_HS1.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_HS1.Init.dma_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_HS1) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief  Write a null-terminated string to CDC interface 0 and flush it.
 *         Needs tusb_init(); data written before enumeration can be lost.
 */
void usb_log(char *msg) {
	tud_cdc_n_write(0, msg, strlen(msg));
	tud_cdc_n_write_flush(0);
	tud_cdc_n_write_flush(0);
	tud_cdc_n_write_flush(0);
}

/**
 *   STX | LEN_L | LEN_H | DATA[LEN] | CHK | ETX
 *
 *   STX          0x02
 *   LEN_L/LEN_H  payload length, 16-bit little-endian
 *   CHK          XOR of the DATA bytes
 *   ETX          0x03
 */
_Bool validate_packet(uint8_t *buf, int len) {
    if (len < 6) return false;  // Minimum: STX + LEN16 + CHK + ETX.
    if (buf[0] != 0x02) return false;
    if (buf[len - 1] != 0x03) return false;

    uint16_t datalen = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    if ((int)(datalen + 5) != len) return false;

    uint8_t chk = 0;
    for (uint16_t i = 0; i < datalen; i++) {
        chk ^= buf[3 + i];
    }

    if (chk != buf[3 + datalen]) return false;
    return true;
}
/**
 * @brief  Erase app_erase_size bytes from app_base_addr, in 64 KB blocks where
 *         aligned, else 4 KB sectors. CDC replies:
 *
 *   ERASE_START
 *   ERASE_PROGRESS 0x<address>   after each erase
 *   ERASE_DONE
 */

void XSPI_EraseAppArea(XSPI_HandleTypeDef *hxspi) {
	XSPI_RegularCmdTypeDef sCommand = { 0 };
	uint32_t address = app_base_addr;  /* Use address from DFU START */
	uint32_t end = app_base_addr + app_erase_size;

	usb_log("ERASE_START\r\n");

	while (address < end) {
	    XSPI_WriteEnable(hxspi);

	    uint32_t step;
	    if (((address % BLOCK_64K_SIZE) == 0) && ((end - address) >= BLOCK_64K_SIZE)) {
	        sCommand.Instruction = OCTAL_BLOCK_ERASE_64K_CMD;
	        step = BLOCK_64K_SIZE;
	    } else {
	        sCommand.Instruction = OCTAL_SECTOR_ERASE_CMD;
	        step = XSPI_SECTOR_SIZE;  /* 4 KB */
	    }

	    sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
	    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
	    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;

	    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
	    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
	    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
	    sCommand.Address = address;

	    sCommand.DataMode = HAL_XSPI_DATA_NONE;
	    sCommand.DummyCycles = 0;
	    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

	    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	        Error_Handler();
	    }

	    XSPI_AutoPollingMemReady(hxspi);

	    char msg[48];
	    snprintf(msg, sizeof(msg), "ERASE_PROGRESS 0x%08lX\r\n", address);
	    usb_log(msg);

	    address += step;

	    tud_task();
	    cdc_task();
	}

	usb_log("ERASE_DONE\r\n");
	tud_task();
	cdc_task();
}

/**
 * @brief  Switch the NOR from single-line SPI to octal DTR through the XSPI.
 */

static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi) {
	uint8_t reg = 0;
	XSPI_RegularCmdTypeDef sCommand = { 0 };
	XSPI_AutoPollingTypeDef sConfig = { 0 };

	sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DummyCycles = 0;
	sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;
	sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
	sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
	sConfig.IntervalTime = 0x10;

	/* Enable write operations */
	sCommand.Instruction = WRITE_ENABLE_CMD;
	sCommand.DataMode = HAL_XSPI_DATA_NONE;
	sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Reconfigure XSPI to automatic polling mode to wait for write enabling */
	sConfig.MatchMask = 0x02;
	sConfig.MatchValue = 0x02;

	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
	sCommand.DataLength = 1;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Write Configuration register 2 (with Octal I/O SPI protocol) */
	sCommand.Instruction = WRITE_CFG_REG_2_CMD;
	sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
	sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;

	sCommand.Address = 0;
	reg = 0x2;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_XSPI_Transmit(hxspi, &reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
	sCommand.DataLength = 1;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief  Octal DTR write enable, then auto-poll the status register until WEL
 *         is set. Required before every erase or program.
 */
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi) {
	XSPI_RegularCmdTypeDef sCommand = { 0 };
	XSPI_AutoPollingTypeDef sConfig = { 0 };

	/* Enable write operations ------------------------------------------ */
	sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.Instruction = OCTAL_WRITE_ENABLE_CMD;
	sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;
	sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	sCommand.DataMode = HAL_XSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Configure automatic polling mode to wait for write enabling ---- */
	sCommand.Instruction = OCTAL_READ_STATUS_REG_CMD;
	sCommand.Address = 0x0;
	sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
	sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
	sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
	sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
	sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
	sCommand.DataLength = 2;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
	sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
	sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
	sConfig.IntervalTime = AUTO_POLLING_INTERVAL;
	sConfig.MatchMask = WRITE_ENABLE_MASK_VALUE;
	sConfig.MatchValue = WRITE_ENABLE_MATCH_VALUE;

	if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  Auto-poll the NOR status register in octal DTR until WIP clears. USB
 *         is not serviced while it waits.
 */
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi) {
	XSPI_RegularCmdTypeDef sCommand = { 0 };
	XSPI_AutoPollingTypeDef sConfig = { 0 };

	/* Configure automatic polling mode to wait for memory ready ------ */
	sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.Instruction = OCTAL_READ_STATUS_REG_CMD;
	sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	sCommand.Address = 0x0;
	sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
	sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
	sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
	sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
	sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
	sCommand.DataLength = 2;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
	sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

	if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
	sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
	sConfig.IntervalTime = AUTO_POLLING_INTERVAL;
	sConfig.MatchMask = MEMORY_READY_MASK_VALUE;
	sConfig.MatchValue = MEMORY_READY_MATCH_VALUE;

	if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		Error_Handler();
	}
}


/**
 * @brief  Program `len` bytes, 1 to MX25UM_PAGE_SIZE and within one page, at
 *         flash offset `address`; `data` must be in RAM, not mapped PSRAM.
 */
static void XSPI_WritePage(uint32_t address, uint8_t *data, uint32_t len)
{
    XSPI_RegularCmdTypeDef sCommand = {0};

    if (len == 0 || len > MX25UM_PAGE_SIZE)
        return;

    /* Enable write operations */
    XSPI_WriteEnable(&hxspi1);

    /* Configure Page Program command */
    sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.Instruction        = OCTAL_PAGE_PROG_CMD;
    sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;

    sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.Address            = address;

    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
    sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataLength         = len;

    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

    if (HAL_XSPI_Command(&hxspi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_XSPI_Transmit(&hxspi1, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Wait until memory is ready */
    XSPI_AutoPollingMemReady(&hxspi1);
}

/* Set while an asset is programmed: Error_Handler then sets g_fsbl_soft_failed
 * and returns instead of halting. */
volatile int g_fsbl_soft_fail = 0;
volatile int g_fsbl_soft_failed = 0;

void Error_Handler(void) {

	if (g_fsbl_soft_fail) {
		/* Best-effort caller: record it and let the caller unwind. */
		g_fsbl_soft_failed = 1;
		return;
	}
	__disable_irq();
	while (1) {
	}

}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif

static int PSRAM_WriteCR(XSPI_HandleTypeDef *hxspi, uint8_t cr, uint32_t addr)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t tx[2] = { cr, cr };  /* Explicit 2-byte payload. */

    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = 0xC0;   // Mode Register Write

    cmd.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
    cmd.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
    cmd.Address            = addr;

    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_8_LINES;
    cmd.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
    cmd.DataLength         = 2;
    cmd.DummyCycles        = 0;      // register writes are latency 1
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined(XSPI_CCR_SIOO)
    cmd.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

    if (HAL_XSPI_Command(hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return -1;

    if (HAL_XSPI_Transmit(hxspi, tx, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return -1;

    return 0;
}

static int PSRAM_GlobalReset(XSPI_HandleTypeDef *hxspi)
{
    XSPI_RegularCmdTypeDef cmd = {0};

    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = 0xFF;   // Global Reset

    /* A 24-bit dummy address phase makes the 4-clock reset frame. */
    cmd.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
    cmd.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
    cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd.Address            = 0x00000000u;

    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_NONE;
    cmd.DataLength         = 0;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined(XSPI_CCR_SIOO)
    cmd.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

    if (HAL_XSPI_Command(hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return -1;

    /* tRST >= 2us; 1ms is safe */
    HAL_Delay(1);
    return 0;
}

static void dfu_feed_data(uint8_t *data, uint32_t len) {
    uint32_t remaining = len;

    while (remaining) {
        uint32_t n = FLASH_PAGE_SIZE - page_fill;
        if (n > remaining) n = remaining;

        memcpy(&page_buf[page_fill], data, n);
        page_fill += n;
        data += n;
        remaining -= n;

        if (page_fill == FLASH_PAGE_SIZE) {
            XSPI_WritePage(app_write_addr, page_buf, FLASH_PAGE_SIZE);
            app_write_addr += FLASH_PAGE_SIZE;
            page_fill = 0;
        }
    }

    bytes_since_ack += len;
    if (bytes_since_ack >= ACK_EVERY_BYTES) {
        usb_log("N6ACK\r\n");
        bytes_since_ack = 0;
    }
}


