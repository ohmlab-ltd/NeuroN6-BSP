/* The simulated Neuro N6 the boot loader's source runs on. See fsbl_sim.h
 * and README.md. Linux only: it puts the flash, the PSRAM and the peripheral
 * registers at the addresses the loader uses, and runs each boot in a child
 * process.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#define _GNU_SOURCE
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "main.h"                 /* the loader's own header: the real HAL types */
#include "stm32n6xx_hal_iwdg.h"
#include "extmem_manager.h"
#include "tusb.h"
#include "n6_ota_layout.h"
#include "fsbl_sim.h"

int fsbl_main(void);              /* FSBL/Core/Src/main.c, built with -Dmain=fsbl_main */
int sim_pty_master(char *name, unsigned long size);       /* fsbl_pty.c */
extern XSPI_HandleTypeDef hxspi1;

sim_t *sim;

#define PERIPH_BASE_SIM  0x40000000ul
#define PERIPH_SIZE_SIM  0x20000000ul

static uint8_t *const g_flash = (uint8_t *)SIM_FLASH_BASE;

/* In the loader. */

static void child_end(int end)
{
  sim->end = end;
  _exit(0);
}

static void advance(uint32_t ms)
{
  sim->now_ms  += ms;
  sim->rtc_ms  += ms;
  sim->tick_ms += ms;
  if (sim->wdg_running && (sim->now_ms >= sim->wdg_deadline_ms)) {
    child_end(SIM_END_WATCHDOG);
  }
}

/* When the host last heard from the loader, or sent it something. */
static uint64_t usb_active_ms;

/* Time in the flash: the host is waiting for an answer, so this does not
 * count as the loader going quiet. */
static void busy(uint32_t ms)
{
  advance(ms);
  usb_active_ms += ms;
}

/* The loader disables interrupts in one place, Error_Handler, and spins. With
 * the watchdog running that lasts until it resets the board; without, for good. */
void sim_irq_disabled(void)
{
  if (sim->wdg_running) {
    const uint64_t left = (sim->wdg_deadline_ms > sim->now_ms) ? (sim->wdg_deadline_ms - sim->now_ms) : 0u;

    sim->now_ms += left;
    sim->rtc_ms += left;
    child_end(SIM_END_WATCHDOG);
  }
  child_end(SIM_END_ERROR);
}

void sim_system_reset(void) { child_end(SIM_END_RESET); }

/* HAL: the parts with nothing to simulate. */

/* A real host's end of the serial stream, if any. It runs in wall-clock time:
 * an erase that takes the simulated board seconds takes the host none. */
static int      pty_fd = -1;
static uint64_t pty_heard_ms;

static uint64_t wall_ms(void)
{
  struct timespec t;

  (void)clock_gettime(CLOCK_MONOTONIC, &t);
  return ((uint64_t)t.tv_sec * 1000u) + ((uint64_t)t.tv_nsec / 1000000u);
}

HAL_StatusTypeDef HAL_Init(void)                                          { return HAL_OK; }
uint32_t          HAL_GetTick(void)                                       { return sim->tick_ms; }

void HAL_Delay(uint32_t Delay)
{
  if (pty_fd >= 0) {
    usleep(Delay * 1000u);
  }
  advance(Delay);
}
void              HAL_NVIC_SystemReset(void)                              { child_end(SIM_END_RESET); for (;;) { } }
void              HAL_PWR_EnableBkUpAccess(void)                          { }
HAL_StatusTypeDef HAL_PWREx_ConfigSupply(uint32_t SupplySource)           { (void)SupplySource; return HAL_OK; }
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling){ (void)VoltageScaling; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_OscConfig(const RCC_OscInitTypeDef *p)          { (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(const RCC_ClkInitTypeDef *p)        { (void)p; return HAL_OK; }
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *p)           { memset(p, 0, sizeof(*p)); }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim)               { (void)htim; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, const TIM_OC_InitTypeDef *c, uint32_t ch)
                                                                          { (void)htim; (void)c; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t ch) { (void)htim; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)                   { (void)hpcd; return HAL_OK; }
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc)                   { (void)hrtc; return HAL_OK; }
HAL_StatusTypeDef HAL_RTCEx_PrivilegeModeSet(const RTC_HandleTypeDef *hrtc, const RTC_PrivilegeStateTypeDef *s)
                                                                          { (void)hrtc; (void)s; return HAL_OK; }
HAL_StatusTypeDef HAL_RTCEx_SecureModeSet(const RTC_HandleTypeDef *hrtc, const RTC_SecureStateTypeDef *s)
                                                                          { (void)hrtc; (void)s; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPI_Init(XSPI_HandleTypeDef *hxspi)                { (void)hxspi; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPIM_Config(XSPI_HandleTypeDef *hxspi, const XSPIM_CfgTypeDef *c, uint32_t t)
                                                                          { (void)hxspi; (void)c; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPI_Abort(XSPI_HandleTypeDef *hxspi)               { (void)hxspi; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPI_SetClockPrescaler(XSPI_HandleTypeDef *hxspi, uint32_t p)
                                                                          { (void)hxspi; (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPI_SetDelayValue(XSPI_HandleTypeDef *hxspi, const XSPI_HSCalTypeDef *c)
                                                                          { (void)hxspi; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_XSPI_MemoryMapped(XSPI_HandleTypeDef *hxspi, const XSPI_MemoryMappedTypeDef *c)
                                                                          { (void)hxspi; (void)c; return HAL_OK; }
void              MX_EXTMEM_MANAGER_Init(void)                            { }

/* The RTC and the backup registers. */

HAL_StatusTypeDef HAL_RTC_GetTime(const RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *t, uint32_t Format)
{
  const uint64_t s = sim->rtc_ms / 1000u;
  (void)hrtc; (void)Format;
  memset(t, 0, sizeof(*t));
  t->Hours   = (uint8_t)((s / 3600u) % 24u);
  t->Minutes = (uint8_t)((s / 60u) % 60u);
  t->Seconds = (uint8_t)(s % 60u);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_RTC_GetDate(const RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *d, uint32_t Format)
{
  (void)hrtc; (void)Format;
  memset(d, 0, sizeof(*d));
  return HAL_OK;
}

volatile uint32_t *sim_bkp(uint32_t idx) { return &(&TAMP->BKP0R)[idx]; }

void HAL_RTCEx_BKUPWrite(const RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data)
{
  (void)hrtc;
  *sim_bkp(BackupRegister) = Data;
}

uint32_t HAL_RTCEx_BKUPRead(const RTC_HandleTypeDef *hrtc, uint32_t BackupRegister)
{
  (void)hrtc;
  return *sim_bkp(BackupRegister);
}

/* The watchdog: LSI at 32 kHz. */

HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
{
  static const uint32_t div[] = { 4u, 8u, 16u, 32u, 64u, 128u, 256u, 512u };
  const uint32_t pr = hiwdg->Init.Prescaler & 0xFu;
  const uint32_t d  = (pr < 8u) ? div[pr] : 1024u;

  sim->wdg_timeout_ms  = (uint32_t)(((uint64_t)(hiwdg->Init.Reload + 1u) * d * 1000u) / 32000u);
  sim->wdg_running     = 1;
  sim->wdg_deadline_ms = sim->now_ms + sim->wdg_timeout_ms;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
  (void)hiwdg;
  if (sim->wdg_running) {
    sim->wdg_deadline_ms = sim->now_ms + sim->wdg_timeout_ms;
  }
  return HAL_OK;
}

/* The NOR, bit-banged: the mode change the loader sends at every start. */

static int     bb_cs = 1, bb_clk, bb_mosi, bb_bits;
static uint8_t bb_bytes[8];

static void bb_end(void)
{
  const int n = bb_bits / 8;

  if (!sim->nor_opi && !sim->bb_deaf) {      /* an octal flash does not hear single-line bytes */
    if ((n == 1) && (bb_bytes[0] == 0x06u)) {
      sim->nor_wel = 1;
    } else if ((n == 6) && (bb_bytes[0] == 0x72u) && sim->nor_wel) {
      if ((bb_bytes[1] | bb_bytes[2] | bb_bytes[3] | bb_bytes[4]) == 0u) {
        sim->nor_opi = (bb_bytes[5] == 0x02u);
      }
      sim->nor_wel = 0;
    }
  }
  bb_bits = 0;
}

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_InitTypeDef *GPIO_Init) { (void)GPIOx; (void)GPIO_Init; }

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  const int v = (PinState != GPIO_PIN_RESET);

  if (GPIOx != GPION) {
    return;
  }
  if (GPIO_Pin & GPIO_PIN_2) { bb_mosi = v; }
  if (GPIO_Pin & GPIO_PIN_1) {
    if (v && !bb_cs) { bb_end(); }
    if (!v && bb_cs) { bb_bits = 0; }
    bb_cs = v;
  }
  if (GPIO_Pin & GPIO_PIN_6) {
    if (v && !bb_clk && !bb_cs) {            /* rising edge: the flash samples MOSI */
      if (bb_bits < (int)(8 * sizeof(bb_bytes))) {
        const int i = bb_bits / 8;
        bb_bytes[i] = (uint8_t)((bb_bytes[i] << 1) | (bb_mosi ? 1u : 0u));
      }
      bb_bits++;
    }
    bb_clk = v;
  }
}

GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  if ((GPIOx == GPION) && (GPIO_Pin & GPIO_PIN_3) && !sim->nor_opi &&
      (bb_bits > 8) && (bb_bytes[0] == 0x05u)) {
    /* the status register, most significant bit first: WEL is bit 1, never busy */
    const int bit = 7 - ((bb_bits - 9) % 8);
    return (sim->nor_wel && (bit == 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  return GPIO_PIN_RESET;                     /* the pull-down, with nothing driving the line */
}

/* The NOR, through XSPI1. */

enum { P_NONE, P_STATUS, P_STATUS_UNHEARD, P_PROGRAM, P_READ, P_WRCR2 };
static int      pend;
static uint32_t pend_addr, pend_len;

static void flash_op(uint8_t kind, uint32_t addr, uint32_t len, uint32_t ms)
{
  if (sim->ops < SIM_OPLOG) {
    sim->oplog[sim->ops].kind = kind;
    sim->oplog[sim->ops].addr = addr;
    sim->oplog[sim->ops].len  = len;
  }
  sim->ops++;
  if (kind == 'P') { sim->programs++; } else { sim->erases++; }
  if ((sim->cut_at_op != 0u) && (sim->ops == sim->cut_at_op)) {
    child_end(SIM_END_POWER_CUT);
  }
  busy(ms);
}

static void nor_erase(uint32_t addr, uint32_t size)
{
  addr &= ~(size - 1u);
  if (!sim->nor_wel) { sim->no_wel++; return; }
  if (addr + size <= SIM_FLASH_SIZE) {
    memset(g_flash + addr, 0xFF, size);
  }
  sim->nor_wel = 0;
  flash_op('E', addr, size, (size == 0x1000u) ? SIM_MS_ERASE_4K : SIM_MS_ERASE_64K);
}

HAL_StatusTypeDef HAL_XSPI_Command(XSPI_HandleTypeDef *hxspi, const XSPI_RegularCmdTypeDef *c, uint32_t Timeout)
{
  (void)Timeout;
  if (hxspi != &hxspi1) {
    return HAL_OK;                           /* the PSRAM's mode registers */
  }
  if (c->OperationType != HAL_XSPI_OPTYPE_COMMON_CFG) {
    return HAL_OK;                           /* the mapping's read and write commands */
  }
  pend = P_NONE;
  if (c->InstructionMode == HAL_XSPI_INSTRUCTION_1_LINE) {
    if (sim->nor_opi) {                      /* the flash is in octal mode and does not hear it */
      sim->spi_in_opi++;
      pend = (c->Instruction == 0x05u) ? P_STATUS_UNHEARD : P_NONE;
      return HAL_OK;
    }
    switch (c->Instruction) {
      case 0x06u: sim->nor_wel = 1; break;
      case 0x05u: pend = P_STATUS; break;
      case 0x72u: pend = P_WRCR2; pend_addr = c->Address; break;
      default: break;
    }
    return HAL_OK;
  }
  if (!sim->nor_opi) {                       /* a flash still in SPI mode does not hear octal commands */
    pend = (c->Instruction == 0x05FAu) ? P_STATUS_UNHEARD : P_NONE;
    return HAL_OK;
  }
  switch (c->Instruction) {
    case 0x06F9u: sim->nor_wel = !sim->nor_locked; break;
    case 0x05FAu: pend = P_STATUS; break;
    case 0x21DEu: nor_erase(c->Address, 0x1000u); break;
    case 0xDC23u: nor_erase(c->Address, 0x10000u); break;
    case 0x12EDu: pend = P_PROGRAM; pend_addr = c->Address; pend_len = c->DataLength; break;
    case 0xEE11u: pend = P_READ;    pend_addr = c->Address; pend_len = c->DataLength; break;
    default: break;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_XSPI_Transmit(XSPI_HandleTypeDef *hxspi, const uint8_t *pData, uint32_t Timeout)
{
  (void)Timeout;
  if (hxspi != &hxspi1) {
    return HAL_OK;
  }
  if (pend == P_WRCR2) {
    if (sim->nor_wel && (pend_addr == 0u)) { sim->nor_opi = (pData[0] == 0x02u); }
    sim->nor_wel = 0;
  } else if (pend == P_PROGRAM) {
    if (!sim->nor_wel) {
      sim->no_wel++;
    } else if (pend_addr + pend_len <= SIM_FLASH_SIZE) {
      for (uint32_t i = 0u; i < pend_len; i++) {
        const uint8_t now = (uint8_t)(g_flash[pend_addr + i] & pData[i]);   /* a program only clears bits */
        if (now != pData[i]) { sim->prog_dirty++; }
        g_flash[pend_addr + i] = now;
      }
      if ((sim->flip_at_program != 0u) && (sim->programs + 1u == sim->flip_at_program)) {
        g_flash[pend_addr] ^= 0x01u;         /* a cell that did not take */
      }
      sim->nor_wel = 0;
      flash_op('P', pend_addr, pend_len, (sim->program_ms != 0u) ? sim->program_ms : SIM_MS_PROGRAM);
    }
  }
  pend = P_NONE;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_XSPI_Receive(XSPI_HandleTypeDef *hxspi, uint8_t *pData, uint32_t Timeout)
{
  (void)Timeout;
  if ((hxspi == &hxspi1) && (pend == P_READ) && (pend_addr + pend_len <= SIM_FLASH_SIZE)) {
    memcpy(pData, g_flash + pend_addr, pend_len);
  }
  pend = P_NONE;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_XSPI_AutoPolling(XSPI_HandleTypeDef *hxspi, const XSPI_AutoPollingTypeDef *p, uint32_t Timeout)
{
  uint32_t status;

  if (hxspi != &hxspi1) {
    return HAL_OK;
  }
  if (pend == P_STATUS) {
    status = sim->nor_wel ? 0x02u : 0x00u;   /* never busy: every operation is done at once */
  } else {
    status = sim->io1_high ? 0xFFu : 0x00u;  /* nothing drives the lines the HAL reads */
  }
  pend = P_NONE;
  if ((status & p->MatchMask) == p->MatchValue) {
    return HAL_OK;
  }
  busy(Timeout);                             /* the loader waits as long as it said it would */
  return HAL_TIMEOUT;
}

EXTMEM_StatusTypeDef EXTMEM_MemoryMappedMode(uint32_t MemId, EXTMEM_StateTypeDef State)
{
  if ((MemId < 2u) && (State == EXTMEM_ENABLE)) {
    const int first = sim->fail_map_call[MemId];
    const int call  = ++sim->map_calls[MemId];

    if ((first != 0) && (call >= first) && (call <= first + sim->fail_map_run[MemId])) {
      return EXTMEM_ERROR_DRIVER;
    }
  }
  return EXTMEM_OK;
}

/* The jump to an application. */

static uint32_t boot_src = N6_OTA_SLOT_A_OFF;

void BOOT_SetSourceOffset(uint32_t offset) { boot_src = offset; }

static int vectors_sane(const uint8_t *image)
{
  uint32_t sp, pc;

  memcpy(&sp, image + 0x400u, 4u);
  memcpy(&pc, image + 0x404u, 4u);
  return ((sp & 3u) == 0u) && (sp > 0x34000000u) && (sp <= 0x34400000u) &&
         ((pc & 1u) != 0u) && (pc >= 0x34000400u) && (pc < 0x34000000u + N6_OTA_MAX_LOADABLE);
}

BOOTStatus_TypeDef BOOT_Application(void)
{
  if (sim->boot_fails) {
    return BOOT_ERROR_MAPPEDMODEFAIL;
  }
  sim->boot_offset = boot_src;
  sim->boot_sane   = vectors_sane(g_flash + boot_src);
  child_end(SIM_END_APP);
  return BOOT_OK;
}

/* The USB serial stream: a host that waits for each answer. */

bool tusb_init(void)
{
  sim->entered_dfu = 1;
  usb_active_ms    = sim->now_ms;
  pty_heard_ms     = wall_ms();
  return true;
}

/* The host sends its next frame once the last is acknowledged, as the
 * uploader does. After a frame left unacknowledged, it carries on once the
 * loader has been idle and silent this long. */
#define HOST_GIVES_UP_MS 5000u

/* A real host, on the pseudo-terminal: what it has sent and the loader has
 * not yet read. */
static uint8_t  pty_in[1u << 16];
static uint32_t pty_len, pty_pos;

static void pty_fill(void)
{
  ssize_t n;

  if (pty_pos == pty_len) {
    pty_pos = pty_len = 0u;
  }
  n = read(pty_fd, pty_in + pty_len, sizeof(pty_in) - pty_len);
  if (n > 0) {                               /* nothing to read, or no host yet, are both nothing */
    pty_len += (uint32_t)n;
    pty_heard_ms = wall_ms();
  }
}

void tud_task(void)
{
  if (pty_fd >= 0) {
    pty_fill();
    if ((pty_pos == pty_len) && ((wall_ms() - pty_heard_ms) >= sim->dfu_idle_limit_ms)) {
      child_end(SIM_END_DFU_IDLE);
    }
    return;
  }
  if (sim->frames_released < sim->frames) {
    if ((sim->frames_released == 0u) || (sim->acks > sim->acks_at_release) ||
        ((sim->now_ms - usb_active_ms) >= HOST_GIVES_UP_MS)) {
      sim->h2d_released    = sim->frame_end[sim->frames_released++];
      sim->acks_at_release = sim->acks;
      sim->release_ms      = sim->now_ms;
      usb_active_ms        = sim->now_ms;
    }
  } else if ((sim->h2d_pos == sim->h2d_len) &&
             ((sim->now_ms - usb_active_ms) >= sim->dfu_idle_limit_ms)) {
    child_end(SIM_END_DFU_IDLE);
  }
}

uint32_t tud_cdc_n_available(uint8_t itf)
{
  (void)itf;
  if (pty_fd >= 0) {
    pty_fill();
    return pty_len - pty_pos;
  }
  return sim->h2d_released - sim->h2d_pos;
}

uint32_t tud_cdc_n_read(uint8_t itf, void *buffer, uint32_t bufsize)
{
  uint32_t n = sim->h2d_released - sim->h2d_pos;

  (void)itf;
  if (pty_fd >= 0) {
    n = pty_len - pty_pos;
    if (n > bufsize) { n = bufsize; }
    memcpy(buffer, pty_in + pty_pos, n);
    pty_pos += n;
    return n;
  }
  if (n > bufsize) { n = bufsize; }
  memcpy(buffer, sim->h2d + sim->h2d_pos, n);
  sim->h2d_pos += n;
  usb_active_ms = sim->now_ms;
  return n;
}

uint32_t tud_cdc_n_write(uint8_t itf, const void *buffer, uint32_t bufsize)
{
  const char *p = (const char *)buffer;

  (void)itf;
  for (uint32_t sent = 0u, spins = 0u; (pty_fd >= 0) && (sent < bufsize) && (spins < 5000u); ) {
    const ssize_t n = write(pty_fd, p + sent, bufsize - sent);

    if (n > 0) { sent += (uint32_t)n; } else { usleep(1000u); spins++; }
  }
  if (sim->d2h_len + bufsize < sizeof(sim->d2h)) {
    memcpy(sim->d2h + sim->d2h_len, buffer, bufsize);
    sim->d2h_len += bufsize;
    sim->d2h[sim->d2h_len] = '\0';
  }
  for (uint32_t i = 0u; i + 5u <= bufsize; i++) {
    if (memcmp(p + i, "N6ACK", 5u) == 0) { sim->acks++; }
  }
  usb_active_ms = sim->now_ms;
  return bufsize;
}

uint32_t tud_cdc_n_write_flush(uint8_t itf) { (void)itf; return 0u; }

/* In the test. */

static void *map_at(unsigned long base, unsigned long size)
{
  void *p = mmap((void *)base, size, PROT_READ | PROT_WRITE,
                 MAP_SHARED | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE | MAP_NORESERVE, -1, 0);

  if (p != (void *)base) {
    perror("fsbl_sim: mmap at a fixed address");
    exit(2);
  }
  return p;
}

uint8_t *sim_flash(uint32_t off)  { return g_flash + off; }
uint8_t *sim_psram(uint32_t addr) { return (uint8_t *)(uintptr_t)addr; }
uint32_t sim_crc32(const void *p, uint32_t len) { return n6_ota_crc32(0u, (const uint8_t *)p, len); }

static void power_on(void)
{
  uint32_t x = 0x12345678u ^ (uint32_t)sim->now_ms;
  uint8_t *ps = (uint8_t *)SIM_PSRAM_BASE;

  memset((void *)TAMP, 0, sizeof(TAMP_TypeDef));         /* the backup domain starts empty */
  for (uint32_t i = 0x01E00000u; i < SIM_PSRAM_SIZE; i += 4u) {   /* the staging area: noise */
    x = x * 1664525u + 1013904223u;
    memcpy(ps + i, &x, 4u);
  }
  sim->rtc_ms      = 0u;
  sim->nor_opi     = 0;                                   /* the NOR powers up in SPI mode */
  sim->nor_wel     = 0;
  sim->wdg_running = 0;
}

void sim_init(void)
{
  static int mapped;

  if (!mapped) {
    sim = mmap(NULL, sizeof(sim_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (sim == MAP_FAILED) { perror("fsbl_sim: mmap"); exit(2); }
    (void)map_at(SIM_FLASH_BASE, SIM_FLASH_SIZE);
    (void)map_at(SIM_PSRAM_BASE, SIM_PSRAM_SIZE);
    (void)map_at(PERIPH_BASE_SIM, PERIPH_SIZE_SIM);
    mapped = 1;
  }
  memset(sim, 0, sizeof(*sim));
  memset(g_flash, 0xFF, SIM_FLASH_SIZE);
  sim->io1_high          = 1;
  sim->dfu_idle_limit_ms = 2000u;
  power_on();
}

void sim_power_cycle(void)
{
  sim->now_ms += 3000u;                                   /* off for a few seconds */
  power_on();
}

int sim_elapse(uint32_t ms)
{
  if (sim->wdg_running && !sim->app_feeds_wdg && ((sim->now_ms + ms) >= sim->wdg_deadline_ms)) {
    const uint64_t d = sim->wdg_deadline_ms - sim->now_ms;
    sim->now_ms += d;
    sim->rtc_ms += d;
    sim->wdg_resets++;
    return 1;                                             /* the watchdog reset the board */
  }
  sim->now_ms += ms;
  sim->rtc_ms += ms;
  return 0;
}

int sim_boot(void)
{
  pid_t pid;
  int   status = 0;

  sim->end           = SIM_END_NONE;
  sim->tick_ms       = 0u;
  sim->wdg_running   = 0;                                 /* a reset stops the watchdog */
  sim->app_feeds_wdg = 0;
  sim->entered_dfu   = 0;
  sim->map_calls[0]  = 0;
  sim->map_calls[1]  = 0;
  sim->d2h_len       = 0u;
  sim->d2h[0]        = '\0';
  sim->acks          = 0u;
  sim->acks_at_release = 0u;
  sim->boots++;

  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    alarm((pty_fd >= 0) ? 600 : 60);                      /* a loader that spins is a hang */
    (void)fsbl_main();
    child_end(SIM_END_HANG);
  }
  (void)waitpid(pid, &status, 0);
  if (WIFSIGNALED(status)) {
    sim->end = (WTERMSIG(status) == SIGALRM) ? SIM_END_HANG : SIM_END_CRASH;
  }
  if (sim->end == SIM_END_WATCHDOG) {
    sim->wdg_resets++;
  }
  return sim->end;
}

int sim_run(int max_boots)
{
  int end = SIM_END_NONE;

  for (int i = 0; i < max_boots; i++) {
    end = sim_boot();
    if ((end != SIM_END_RESET) && (end != SIM_END_WATCHDOG)) {
      break;
    }
  }
  return end;
}

/* The host's side of the serial stream. */

int sim_pty_open(char *name, unsigned long size)
{
  pty_fd = sim_pty_master(name, size);
  return (pty_fd >= 0) ? 0 : -1;
}

const char *sim_end_name(int end)
{
  static const char *const names[] = { "NONE", "APP", "RESET", "DFU_IDLE", "ERROR", "HANG",
                                       "WATCHDOG", "POWER_CUT", "CRASH" };

  return ((end >= 0) && (end <= SIM_END_CRASH)) ? names[end] : "?";
}

void sim_host_clear(void)
{
  sim->h2d_len = sim->h2d_pos = sim->h2d_released = 0u;
  sim->frames = sim->frames_released = 0u;
}

void sim_host_raw(const void *bytes, uint32_t len)
{
  if ((sim->h2d_len + len <= sizeof(sim->h2d)) && (sim->frames < 4096u)) {
    memcpy(sim->h2d + sim->h2d_len, bytes, len);
    sim->h2d_len += len;
    sim->frame_end[sim->frames++] = sim->h2d_len;
  }
}

void sim_host_frame(const void *payload, uint32_t len)
{
  static uint8_t f[8192 + 8];
  const uint8_t *p = (const uint8_t *)payload;
  uint8_t x = 0u;

  f[0] = 0x02u;
  f[1] = (uint8_t)(len & 0xFFu);
  f[2] = (uint8_t)(len >> 8);
  for (uint32_t i = 0u; i < len; i++) { f[3u + i] = p[i]; x ^= p[i]; }
  f[3u + len] = x;
  f[4u + len] = 0x03u;
  sim_host_raw(f, len + 5u);
}

int sim_reply_count(const char *line)
{
  const size_t n = strlen(line);
  const char *p = sim->d2h;
  int count = 0;

  while ((p = strstr(p, line)) != NULL) {
    if (((p == sim->d2h) || (p[-1] == '\n')) && (p[n] == '\r')) { count++; }
    p += n;
  }
  return count;
}

int sim_reply_has(const char *line) { return sim_reply_count(line) > 0; }

static uint32_t ops_in(uint8_t kind, uint32_t off, uint32_t len)
{
  const uint32_t n = (sim->ops < SIM_OPLOG) ? sim->ops : SIM_OPLOG;
  uint32_t count = 0u;

  for (uint32_t i = 0u; i < n; i++) {
    if ((sim->oplog[i].kind == kind) && (sim->oplog[i].addr < off + len) &&
        (sim->oplog[i].addr + sim->oplog[i].len > off)) {
      count++;
    }
  }
  return count;
}

void sim_ops_clear(void)
{
  sim->ops = sim->erases = sim->programs = 0u;
  sim->spi_in_opi = sim->prog_dirty = sim->no_wel = 0u;
}

uint32_t sim_erased_in(uint32_t off, uint32_t len)   { return ops_in('E', off, len); }
uint32_t sim_programs_in(uint32_t off, uint32_t len) { return ops_in('P', off, len); }
