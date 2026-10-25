/* The boot loader's source, FSBL/Core/Src/main.c, run on a simulated board.
 *
 * Each test sets a board up, runs the loader through one or more boots, and
 * says what must be true afterwards. See README.md for what the simulation
 * covers and what it cannot.
 *
 *   sh harness/bench/hosttest/fsbl/run.sh        (Linux, or WSL on Windows)
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "n6_ota_layout.h"
#include "fsbl_sim.h"

#define FORCEDFU   0x00F4CDF4u            /* the loader's own key, in backup register 2 */

/* Result codes and the trial watchdog period that the open findings' tests
 * expect (docs/BOOT-CONTRACT.md, "Known issues in v2.6.2"); v2.6.2's layout
 * does not define them. */
#ifndef N6_OTA_APPLY_ERR_FLASH_READ
#define N6_OTA_APPLY_ERR_FLASH_READ 0x45520000u   /* flash unreadable; the slot is untouched */
#endif
#ifndef N6_OTA_APPLY_ERR_NOT_APP
#define N6_OTA_APPLY_ERR_NOT_APP    0x45560000u   /* not an application for its slot */
#endif
#ifndef N6_ASSET_RESULT_ERR_READ
#define N6_ASSET_RESULT_ERR_READ    6u            /* a stored copy could not be read */
#endif
#ifndef N6_OTA_TRIAL_WDG_MS
#define N6_OTA_TRIAL_WDG_MS         32768u        /* the watchdog an image on trial runs under */
#endif
#define SLOT_A     N6_OTA_SLOT_A_OFF
#define SLOT_B     N6_OTA_SLOT_B_OFF

static int g_fail;                        /* failures in the test running now */

#define CHECK(c) do { if (!(c)) { g_fail++; printf("      line %d: %s\n", __LINE__, #c); } } while (0)

/* Images. */

static uint8_t g_img[3][N6_OTA_SLOT_SIZE];

static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4u); }

/* An application image as the build makes one: the header's magic, and
 * a vector table 0x400 in whose reset vector points into the image. */
static uint8_t *make_app(int which, uint32_t size, uint32_t seed)
{
  uint8_t *img = g_img[which];
  uint32_t x = seed * 2654435761u + 12345u;

  for (uint32_t i = 0u; i < size; i++) {
    x = x * 1664525u + 1013904223u;
    img[i] = (uint8_t)(x >> 24);
  }
  memcpy(img, "STM2", 4u);
  put32(img + 0x400u, 0x34200000u);
  put32(img + 0x404u, 0x34000401u + ((seed & 0xFFu) << 8));
  return img;
}

/* A boot loader image: the same header, a reset vector in the loader's RAM. */
static uint8_t *make_fsbl(int which, uint32_t size, uint32_t seed)
{
  uint8_t *img = make_app(which, size, seed);

  put32(img + 0x404u, 0x34184001u + ((seed & 0xFFu) << 4));
  return img;
}

static void flash_put(uint32_t off, const uint8_t *p, uint32_t size)
{
  memcpy(sim_flash(off), p, size);
}

static int flash_is(uint32_t off, const uint8_t *p, uint32_t size)
{
  return memcmp(sim_flash(off), p, size) == 0;
}

static int flash_blank(uint32_t off, uint32_t size)
{
  for (uint32_t i = 0u; i < size; i++) {
    if (*sim_flash(off + i) != 0xFFu) { return 0; }
  }
  return 1;
}

/* Control block. */

static int cb_read(n6_ota_cb_t *out)
{
  n6_ota_cb_t c[2];
  int v0, v1, win;

  memcpy(&c[0], sim_flash(N6_OTA_CB_OFF(0)), sizeof(c[0]));
  memcpy(&c[1], sim_flash(N6_OTA_CB_OFF(1)), sizeof(c[1]));
  v0 = n6_ota_cb_valid(&c[0]);
  v1 = n6_ota_cb_valid(&c[1]);
  if (!v0 && !v1) { return -1; }
  win = (v0 && v1) ? ((c[1].seq > c[0].seq) ? 1 : 0) : (v1 ? 1 : 0);
  *out = c[win];
  return win;
}

static void cb_write(int copy, n6_ota_cb_t *cb)
{
  cb->magic = N6_OTA_CB_MAGIC;
  cb->crc   = n6_ota_cb_crc(cb);
  memset(sim_flash(N6_OTA_CB_OFF(copy)), 0xFF, N6_OTA_CB_SECTOR);
  memcpy(sim_flash(N6_OTA_CB_OFF(copy)), cb, sizeof(*cb));
}

static void cb_slot(n6_ota_cb_t *cb, int slot, const uint8_t *img, uint32_t size, uint32_t state)
{
  cb->slot[slot].crc32 = sim_crc32(img, size);
  cb->slot[slot].size  = size;
  cb->slot[slot].state = state;
}

/* A board that has been updated before: both slots hold confirmed images. */
static void board_ab(uint32_t size, uint8_t active)
{
  n6_ota_cb_t cb;

  sim_init();
  flash_put(SLOT_A, make_app(0, size, 1u), size);
  flash_put(SLOT_B, make_app(1, size, 2u), size);
  memset(&cb, 0, sizeof(cb));
  cb.seq     = 5u;
  cb.active  = active;
  cb.pending = N6_OTA_NO_PENDING;
  cb_slot(&cb, 0, g_img[0], size, N6_OTA_SLOT_CONFIRMED);
  cb_slot(&cb, 1, g_img[1], size, N6_OTA_SLOT_CONFIRMED);
  cb_write(0, &cb);
}

static void app_stage_image(const uint8_t *img, uint32_t size, uint32_t version);
static int  reset_after(uint32_t ms);

/* A board running the slot `active`, whose application has staged an update
 * and reset: the new image is on the first boot of its trial. */
static uint8_t *board_on_trial(uint8_t active, uint32_t size, uint32_t seed)
{
  uint8_t *img = make_app(2, size, seed);

  board_ab(200000u, active);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == (active ? SLOT_B : SLOT_A));
  app_stage_image(img, size, 3u);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == (active ? SLOT_A : SLOT_B));
  return img;
}

/* What an application does. */

static void app_stage_image(const uint8_t *img, uint32_t size, uint32_t version)
{
  n6_ota_psram_hdr_t h;

  h.magic   = N6_OTA_PSRAM_HDR_MAGIC;
  h.size    = size;
  h.version = version;
  h.crc32   = sim_crc32(img, size);
  h.hdr_crc = sim_crc32(&h, 16u);
  memcpy(sim_psram(N6_OTA_PSRAM_BASE + N6_OTA_PSRAM_IMAGE_OFF), img, size);
  memcpy(sim_psram(N6_OTA_PSRAM_BASE), &h, sizeof(h));
  *sim_bkp(N6_OTA_TRIGGER_BKPR) = N6_OTA_TRIGGER_MAGIC;
}

static void app_stage_asset(const void *data, uint32_t len, uint32_t version)
{
  n6_asset_hdr_t h;

  h.magic   = N6_ASSET_PSRAM_MAGIC;
  h.version = version;
  h.len     = len;
  h.crc32   = sim_crc32(data, len);
  h.hdr_crc = sim_crc32(&h, 16u);
  memcpy(sim_psram(N6_ASSET_PSRAM_BASE + N6_ASSET_HDR_BYTES), data, len);
  memcpy(sim_psram(N6_ASSET_PSRAM_BASE), &h, sizeof(h));
  *sim_bkp(N6_ASSET_TRIGGER_BKPR) = N6_ASSET_TRIGGER_MAGIC;
}

static void app_confirm(void) { *sim_bkp(N6_OTA_CONFIRM_BKPR) = N6_OTA_CONFIRM_MAGIC; }

/* The application ran well for a while, then the board was reset. */
static int reset_after(uint32_t ms)
{
  sim->app_feeds_wdg = 1;
  (void)sim_elapse(ms);
  return sim_boot();
}

/* The asset copy `c` in flash: its version if it is whole, else 0. */
static uint32_t asset_version(int c)
{
  n6_asset_hdr_t h;

  memcpy(&h, sim_flash(N6_ASSET_OFF(c)), sizeof(h));
  if ((h.magic != N6_ASSET_MAGIC) || (h.hdr_crc != sim_crc32(&h, 16u))) { return 0u; }
  if ((h.len == 0u) || (h.len > N6_ASSET_MAX_BYTES)) { return 0u; }
  if (sim_crc32(sim_flash(N6_ASSET_OFF(c) + N6_ASSET_HDR_BYTES), h.len) != h.crc32) { return 0u; }
  return h.version;
}

/* A host on USB. */

static void dfu_enter(void)
{
  sim_host_clear();
  *sim_bkp(2) = FORCEDFU;
}

static void dfu_cmd32(const char *name, uint32_t a)
{
  uint8_t f[16];
  const uint32_t n = (uint32_t)strlen(name);

  memcpy(f, name, n);
  memcpy(f + n, &a, 4u);
  sim_host_frame(f, n + 4u);
}

static void dfu_text(const char *name) { sim_host_frame(name, (uint32_t)strlen(name)); }

static void dfu_crc(uint32_t off, uint32_t len)
{
  uint8_t f[17];

  memcpy(f, "DFU_CRC32", 9u);
  memcpy(f + 9, &off, 4u);
  memcpy(f + 13, &len, 4u);
  sim_host_frame(f, 17u);
}

static void dfu_blocks(const uint8_t *p, uint32_t size)
{
  for (uint32_t i = 0u; i < size; i += 4096u) {
    sim_host_frame(p + i, (size - i > 4096u) ? 4096u : (size - i));
  }
}

/* One image, the way the uploader sends it. */
static void dfu_image(uint32_t off, const uint8_t *p, uint32_t size)
{
  dfu_cmd32("DFU START", off);
  dfu_cmd32("DFU_ERASE", (size + 0xFFFu) & ~0xFFFu);
  dfu_text("DFU_DATA");
  dfu_blocks(p, size);
  dfu_text("DFU_NEXT");
}

static int reply_crc(uint32_t crc)
{
  char line[32];

  snprintf(line, sizeof(line), "CRC32 0x%08lX", (unsigned long)crc);
  return sim_reply_has(line);
}

/* What works now. */

static void t_boot_legacy(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  CHECK(sim_boot() == SIM_END_APP);
  CHECK(sim->boot_offset == SLOT_A && sim->boot_sane);
  CHECK(!sim->entered_dfu);
  CHECK(sim->ops == 0u);                               /* a plain boot writes nothing */
  CHECK(!sim->wdg_running);                            /* and starts no watchdog */
}

static void t_double_tap(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  CHECK(sim_boot() == SIM_END_APP);
  CHECK(sim_boot() == SIM_END_DFU_IDLE && sim->entered_dfu);     /* reset again at once */
  CHECK(reset_after(3000u) == SIM_END_APP);                      /* and a reset leaves it */
  CHECK(reset_after(3000u) == SIM_END_APP);                      /* a slow second press does nothing */
}

static void t_forced_dfu(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  dfu_enter();
  dfu_text("DFU_INFO");
  CHECK(sim_boot() == SIM_END_DFU_IDLE && sim->entered_dfu);
  CHECK(strstr(sim->d2h, "N6INFO 2 CRC32 MULTI OTA-AB LOAD1536K") != NULL);
  CHECK(strstr(sim->d2h, " UID ") != NULL);
  CHECK(sim_reply_count("N6ACK") == 1);
  CHECK(*sim_bkp(2) == 0u);                            /* the key is used once */
}

static void t_dfu_upload(void)
{
  const uint32_t size = 300000u;
  uint8_t *img = make_app(0, size, 7u);

  sim_init();
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  dfu_crc(SLOT_A, size);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(flash_is(SLOT_A, img, size));
  CHECK(sim_reply_has("NEXT_OK") && reply_crc(sim_crc32(img, size)) && sim_reply_has("RESETTING"));
  CHECK(sim->prog_dirty == 0u && sim->no_wel == 0u);
  CHECK(sim_erased_in(0u, SLOT_A) == 0u);              /* nothing below the slot */
  CHECK(flash_blank(N6_OTA_CB_OFF(0), 0x2000u));       /* no control block on a board without one */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A && sim->boot_sane);
}

static void t_dfu_weights_and_app(void)
{
  const uint32_t wsize = 700001u, asize = 150000u;
  uint8_t *w = make_app(1, wsize, 9u), *a = make_app(0, asize, 8u);

  sim_init();
  dfu_enter();
  dfu_image(0x00300000u, w, wsize);
  dfu_crc(0x00300000u, wsize);
  dfu_image(SLOT_A, a, asize);
  dfu_crc(SLOT_A, asize);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(flash_is(0x00300000u, w, wsize) && flash_is(SLOT_A, a, asize));
  CHECK(sim_reply_count("NEXT_OK") == 2 && reply_crc(sim_crc32(w, wsize)) && reply_crc(sim_crc32(a, asize)));
  CHECK(sim->prog_dirty == 0u);
}

static void t_dfu_bad_requests(void)
{
  sim_init();
  dfu_enter();
  dfu_cmd32("DFU START", 0x00100800u);                 /* not on a sector */
  dfu_cmd32("DFU START", 0x02000000u);                 /* past the flash */
  dfu_cmd32("DFU START", SLOT_A);
  dfu_cmd32("DFU_ERASE", 0x00181000u);                 /* out of the slot, into the control block */
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(sim_reply_count("START_BAD_ADDR") == 2 && sim_reply_has("ERASE_BAD_SIZE"));
  CHECK(sim->erases == 0u && sim->programs == 0u);
}

static void t_dfu_notes_slot(void)
{
  const uint32_t size = 180000u;
  uint8_t *img = make_app(2, size, 30u);
  n6_ota_cb_t cb;

  board_ab(200000u, 1u);                               /* running from B */
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(sim_reply_has("OTA_CB_UPDATED"));
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(cb.slot[0].state == N6_OTA_SLOT_CONFIRMED && cb.slot[0].size == size);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);
}

/* An update and settings the application staged and armed, then an upload
 * over USB into the slot: the upload cancels both, so neither is applied
 * over it at the next boot. */
static void t_dfu_cancels_staged(void)
{
  const uint32_t size = 200000u;
  uint8_t *img = make_app(0, size, 42u);

  sim_init();                                          /* no control block */
  flash_put(SLOT_A, make_app(1, size, 40u), size);
  app_stage_image(make_app(2, size, 41u), size, 5u);
  app_stage_asset("staged settings", 15u, 3u);
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(*sim_bkp(N6_OTA_TRIGGER_BKPR) == 0u && *sim_bkp(N6_ASSET_TRIGGER_BKPR) == 0u);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A && flash_is(SLOT_A, img, size));
  CHECK(flash_blank(N6_OTA_CB_OFF(0), 0x2000u));       /* no update applied, no record made */
  CHECK(asset_version(0) == 0u && asset_version(1) == 0u);
}

/* The same on a board with a control block: the upload is recorded, and what
 * the application staged is cancelled. */
static void t_dfu_cancels_staged_ab(void)
{
  const uint32_t size = 180000u;
  uint8_t *img;
  n6_ota_cb_t cb;

  board_ab(200000u, 1u);                               /* running from B */
  app_stage_image(make_app(2, size, 51u), size, 4u);
  app_stage_asset("staged settings", 15u, 3u);
  img = make_app(0, size, 52u);
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET && sim_reply_has("OTA_CB_UPDATED"));
  CHECK(*sim_bkp(N6_OTA_TRIGGER_BKPR) == 0u && *sim_bkp(N6_ASSET_TRIGGER_BKPR) == 0u);
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A && flash_is(SLOT_A, img, size));
  CHECK(asset_version(0) == 0u && asset_version(1) == 0u);
}

/* An upload that fills no slot, weights alone, leaves what the application
 * staged armed. */
static void t_dfu_weights_keep_staged(void)
{
  const uint32_t size = 200000u, wsize = 300001u;
  uint8_t *w = make_app(1, wsize, 9u);

  sim_init();
  flash_put(SLOT_A, make_app(0, size, 40u), size);
  app_stage_image(make_app(2, size, 41u), size, 5u);
  app_stage_asset("staged settings", 15u, 3u);
  dfu_enter();
  dfu_image(0x00300000u, w, wsize);
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(*sim_bkp(N6_OTA_TRIGGER_BKPR) == N6_OTA_TRIGGER_MAGIC);
  CHECK(*sim_bkp(N6_ASSET_TRIGGER_BKPR) == N6_ASSET_TRIGGER_MAGIC);
}

static void t_update_confirm(void)
{
  const uint32_t size = 400000u;
  uint8_t *img = make_app(2, size, 40u);
  n6_ota_cb_t cb;

  board_ab(200000u, 0u);                               /* running from A */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  app_stage_image(img, size, 3u);
  CHECK(reset_after(2000u) == SIM_END_APP);            /* applied, and booted on trial */
  CHECK(sim->boot_offset == SLOT_B && flash_is(SLOT_B, img, size));
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_OK);
  CHECK(flash_is(SLOT_A, g_img[0], 200000u));          /* the old image is untouched */
  CHECK(cb_read(&cb) >= 0 && cb.pending == 1u && cb.tries_left == N6_OTA_DEFAULT_TRIES - 1u);
  CHECK(cb.active == 0u && cb.slot[1].state == N6_OTA_SLOT_VALID && cb.slot[1].version == 3u);
  app_confirm();
  CHECK(reset_after(5000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(cb.slot[1].state == N6_OTA_SLOT_CONFIRMED);
  CHECK(*sim_bkp(N6_OTA_CONFIRM_BKPR) == 0u);
  CHECK(sim->prog_dirty == 0u && sim->no_wel == 0u);
}

static void t_update_rolls_back(void)
{
  n6_ota_cb_t cb;

  (void)board_on_trial(0u, 400000u, 41u);                                   /* try 1 */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);   /* try 2 */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);   /* try 3 */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);   /* given up */
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);   /* and stays */
}

static void t_update_refuses_bad_stage(void)
{
  const uint32_t size = 300000u;
  uint8_t *img = make_app(2, size, 42u);

  board_ab(200000u, 0u);
  app_stage_image(img, size, 3u);
  *sim_psram(N6_OTA_PSRAM_BASE + N6_OTA_PSRAM_IMAGE_OFF + 1234u) ^= 0x10u;
  sim_ops_clear();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_IMAGE_CRC);
  CHECK(sim->ops == 0u && flash_is(SLOT_B, g_img[1], 200000u));
  CHECK(*sim_bkp(N6_OTA_TRIGGER_BKPR) == 0u);
}

static void t_first_update(void)
{
  const uint32_t size = 350000u;
  uint8_t *img = make_app(2, size, 43u);
  n6_ota_cb_t cb;

  sim_init();                                          /* never updated: no control block */
  flash_put(SLOT_A, make_app(0, 250000u, 1u), 250000u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  app_stage_image(img, size, 2u);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(cb_read(&cb) >= 0 && cb.slot[0].state == N6_OTA_SLOT_CONFIRMED);
  CHECK(cb.slot[0].size == N6_OTA_MAX_LOADABLE);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);   /* unconfirmed: back to A */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);
}

static void t_asset(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  app_stage_asset("first settings", 14u, 1u);
  CHECK(sim_boot() == SIM_END_APP && *sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_OK);
  CHECK(asset_version(0) == 1u && asset_version(1) == 0u);
  app_stage_asset("second settings", 15u, 2u);
  CHECK(reset_after(2000u) == SIM_END_APP && *sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_OK);
  CHECK(asset_version(0) == 1u && asset_version(1) == 2u);            /* the other copy */
  app_stage_asset("third settings", 14u, 3u);
  CHECK(reset_after(2000u) == SIM_END_APP);
  CHECK(asset_version(0) == 3u && asset_version(1) == 2u);
  CHECK(sim->prog_dirty == 0u);
}

static void t_stage_overlap(void)
{
  uint8_t *img = make_app(2, N6_OTA_MAX_LOADABLE, 44u);

  board_ab(200000u, 0u);
  app_stage_image(img, N6_OTA_MAX_LOADABLE, 3u);       /* a full slot reaches the asset's stage */
  app_stage_asset("settings", 8u, 1u);                 /* which lands on the image's last bytes */
  sim_ops_clear();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_IMAGE_CRC);
  CHECK(flash_is(SLOT_B, g_img[1], 200000u) && sim_erased_in(SLOT_B, N6_OTA_SLOT_SIZE) == 0u);
  CHECK(asset_version(0) == 1u);                       /* the asset itself is whole */
}

static void t_stale_record_still_boots(void)
{
  n6_ota_cb_t cb;

  board_ab(200000u, 0u);
  flash_put(SLOT_A, make_app(2, 220000u, 50u), 220000u);    /* written by a tool that keeps no record */
  CHECK(cb_read(&cb) >= 0);
  cb.slot[1].state = N6_OTA_SLOT_EMPTY;                /* and nothing else to fall back on */
  cb_write(0, &cb);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A && sim->boot_sane);
  CHECK(!sim->wdg_running);
}

static void t_mode_switch_missed(void)
{
  const uint32_t size = 100000u;
  uint8_t *img = make_app(0, size, 60u);

  sim_init();
  sim->bb_deaf = 1;                                    /* the start-up mode change did not take */
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(SLOT_A, img, size) && sim_reply_has("NEXT_OK"));
}

/* Findings F1 to F11. Each test states what must hold once its finding is
 * fixed; the list at the end marks those still open (docs/BOOT-CONTRACT.md,
 * "Known issues in v2.6.2"). */

/* F1: an update that works, on a board only ever switched off and on. */
static void t_f1(void)
{
  n6_ota_cb_t cb;

  (void)board_on_trial(0u, 300000u, 70u);
  for (int i = 0; i < 6; i++) {
    app_confirm();                                     /* lost with the power */
    sim->app_feeds_wdg = 1;
    (void)sim_elapse(60000u);
    sim_power_cycle();
    CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  }
  CHECK(cb_read(&cb) >= 0 && cb.pending == 1u && cb.tries_left == N6_OTA_DEFAULT_TRIES - 1u);
  app_confirm();
  CHECK(reset_after(5000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == N6_OTA_NO_PENDING);
}

/* F1: a boot after a power cut neither uses a try nor gives one back. */
static void t_f1_mixed(void)
{
  n6_ota_cb_t cb;

  (void)board_on_trial(0u, 300000u, 70u);                                   /* try 1 */
  sim->app_feeds_wdg = 1;
  (void)sim_elapse(4000u);
  sim_power_cycle();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);           /* not counted */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);   /* try 2 */
  sim->app_feeds_wdg = 1;
  (void)sim_elapse(4000u);
  sim_power_cycle();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);           /* not counted */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);   /* try 3 */
  CHECK(cb_read(&cb) >= 0 && cb.pending == 1u && cb.tries_left == 0u);
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);   /* given up */
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
}

/* F1: the loader's mark is in a register no application writes: a trial image
 * that fills every backup register applications use still has its resets
 * counted. */
static void t_f1_app_registers(void)
{
  static const uint8_t used[] = { 0, 3, 4, 5, 6, 7, 8, 9, 10, 17, 18, 19, 23, 24, 25, 26, 27, 28, 29, 30, 31 };
  n6_ota_cb_t cb;
  int end = SIM_END_APP;

  (void)board_on_trial(0u, 300000u, 70u);                                   /* try 1 */
  for (int i = 0; (i < 8) && (end == SIM_END_APP) && (sim->boot_offset == SLOT_B); i++) {
    for (unsigned r = 0; r < sizeof(used); r++) { *sim_bkp(used[r]) = 0xDEAD0000u + r; }
    end = reset_after(2000u);
  }
  CHECK(end == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(sim->boots == 5u);                             /* one boot, three tries, and the old image */
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
}

/* F2: a trial image that faults asks for the bootloader, as the BSP's fault
 * handler does, and nobody is there. */
static void t_f2(void)
{
  n6_ota_cb_t cb;
  int end = SIM_END_APP;

  (void)board_on_trial(0u, 300000u, 71u);
  sim->dfu_idle_limit_ms = 600000u;                    /* ten minutes with no host */
  for (int i = 0; (i < 8) && (end == SIM_END_APP) && (sim->boot_offset == SLOT_B); i++) {
    sim->app_feeds_wdg = 1;
    (void)sim_elapse(1500u);
    *sim_bkp(2) = FORCEDFU;
    end = sim_run(4);
  }
  CHECK(end == SIM_END_APP && sim->boot_offset == SLOT_A);     /* back on the old image, by itself */
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
}

/* F2: the same for a trial image that resets within a second of starting,
 * which the loader takes for a second press of the reset button. */
static void t_f2_tap(void)
{
  n6_ota_cb_t cb;
  int end = SIM_END_APP;

  (void)board_on_trial(0u, 300000u, 71u);
  sim->dfu_idle_limit_ms = 600000u;
  for (int i = 0; (i < 8) && (end == SIM_END_APP) && (sim->boot_offset == SLOT_B); i++) {
    end = sim_run(4);                                  /* reset at once */
  }
  CHECK(end == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
}

/* F2: a host that does speak keeps the bootloader, trial or not. */
static void t_f2_host(void)
{
  (void)board_on_trial(0u, 300000u, 72u);
  sim->app_feeds_wdg = 1;
  (void)sim_elapse(1500u);
  dfu_enter();
  dfu_text("DFU_INFO");
  sim->dfu_idle_limit_ms = 600000u;
  CHECK(sim_boot() == SIM_END_DFU_IDLE);               /* still there ten minutes later */
}

/* F2: and with no update on trial the bootloader waits as long as it takes. */
static void t_f2_stay(void)
{
  board_ab(200000u, 1u);
  CHECK(sim_boot() == SIM_END_APP);
  sim->app_feeds_wdg = 1;
  (void)sim_elapse(1500u);
  dfu_enter();
  sim->dfu_idle_limit_ms = 600000u;
  CHECK(sim_boot() == SIM_END_DFU_IDLE && sim->entered_dfu);
}

/* F3: a trial image that hangs before it starts a watchdog of its own. */
static void t_f3(void)
{
  int end = SIM_END_APP;

  (void)board_on_trial(0u, 300000u, 73u);
  for (int i = 0; (i < 8) && (end == SIM_END_APP) && (sim->boot_offset == SLOT_B); i++) {
    CHECK(sim->wdg_running && sim->wdg_timeout_ms == N6_OTA_TRIAL_WDG_MS);   /* the loader left one running */
    if (!sim->wdg_running) { return; }
    CHECK(sim_elapse(3600000u) == 1);                  /* which resets the hung image */
    end = sim_boot();
  }
  CHECK(end == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(!sim->wdg_running);                            /* and a confirmed image boots without it */
}

/* F3: the watchdog does not cut short the longest apply. */
static void t_f3_apply(void)
{
  uint8_t *img = board_on_trial(0u, N6_OTA_MAX_LOADABLE, 74u);

  CHECK(flash_is(SLOT_B, img, N6_OTA_MAX_LOADABLE) && sim->wdg_resets == 0u);
  CHECK(sim->wdg_running);
}

/* F3: nor on a flash that programs several times slower than the part's
 * worst case. */
static void t_f3_slow(void)
{
  uint8_t *img = make_app(2, N6_OTA_MAX_LOADABLE, 74u);

  board_ab(200000u, 0u);
  CHECK(sim_boot() == SIM_END_APP);
  app_stage_image(img, N6_OTA_MAX_LOADABLE, 3u);
  sim->program_ms = 8u;                                /* 49 s of programming */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(flash_is(SLOT_B, img, N6_OTA_MAX_LOADABLE) && sim->wdg_resets == 0u);
}

/* F3: an image on trial that confirms but never refreshes the watchdog is
 * reset by it once, and that reset commits the update. */
static void t_f3_confirms(void)
{
  n6_ota_cb_t cb;

  (void)board_on_trial(0u, 300000u, 73u);
  app_confirm();
  CHECK(sim_elapse(3600000u) == 1);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B && !sim->wdg_running);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(sim_elapse(3600000u) == 0);                    /* and it runs on, with no watchdog */
}

/* F3: a flash write that hangs during the apply ends in a reset, and the
 * image that staged the update boots again, with no watchdog running. */
static void t_f3_spin(void)
{
  board_ab(200000u, 0u);
  CHECK(sim_boot() == SIM_END_APP);
  app_stage_image(make_app(2, 300000u, 73u), 300000u, 3u);
  sim->nor_locked = 1;                                 /* the flash takes no writes: the loader stops dead */
  CHECK(reset_after(2000u) == SIM_END_WATCHDOG);
  sim->nor_locked = 0;
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A && !sim->wdg_running);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_STARTED);
}

/* F3: an update that does not check out once written leaves the watchdog
 * running, so the old image is entered only after a reset has stopped it. */
static void t_f3_bad_write(void)
{
  board_ab(200000u, 0u);
  CHECK(sim_boot() == SIM_END_APP);
  app_stage_image(make_app(2, 300000u, 73u), 300000u, 3u);
  sim->flip_at_program = sim->programs + 100u;         /* one page does not hold what was written */
  CHECK(reset_after(2000u) == SIM_END_RESET);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_FLASH_CRC);
  /* As if the write had taken no time at all: the loader's own reset is not
   * a second press of the button. */
  *sim_bkp(1) = (uint32_t)(((sim->rtc_ms / 1000u) % 86400u) * 1000u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A && !sim->wdg_running);
  CHECK(sim_elapse(3600000u) == 0);
}

/* F4: what is in a slot is checked before the jump. */
static void t_f4_garbage(void)
{
  static const uint32_t bad[][2] = {
    { 0xDEADBEEFu, 0x12345678u },
    { 0x00000000u, 0x34000401u },        /* no stack */
    { 0x34200002u, 0x34000401u },        /* a stack pointer off a word boundary */
    { 0x90100000u, 0x34000401u },        /* a stack in the flash */
    { 0x50000000u, 0x34000401u },        /* in the peripherals */
    { 0x34200000u, 0x34000400u },        /* a reset vector not in Thumb state */
    { 0x34200000u, 0x340003FFu },        /* below the image's vectors */
    { 0x34200000u, 0x34180001u },        /* past the window the slot is copied to */
    { 0x34200000u, 0x91C80401u },        /* in the other slot's flash */
    { 0x34200000u, 0x90000401u },        /* in the boot loader's flash */
    { 0x34200000u, 0xFFFFFFFFu },
  };

  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    sim_init();
    flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
    put32(sim_flash(SLOT_A + 0x400u), bad[i][0]);
    put32(sim_flash(SLOT_A + 0x404u), bad[i][1]);
    CHECK(sim_run(3) == SIM_END_DFU_IDLE && sim->entered_dfu);
    if (g_fail != 0) { printf("      vectors %08lX %08lX\n", (unsigned long)bad[i][0], (unsigned long)bad[i][1]); return; }
  }
}

/* F4: and every application that can run is still booted. */
static void t_f4_accepts(void)
{
  static const uint32_t good[][2] = {
    { 0x34200000u, 0x34000401u },        /* the first address after the vectors' own */
    { 0x34200000u, 0x3417FFFFu },        /* the last in the window */
    { 0x30020000u, 0x34010469u },        /* a stack in the DTCM */
    { 0x24200000u, 0x34010469u },        /* by the other alias */
    { 0x343C0000u, 0x34010469u },        /* at the top of the last AXI RAM */
    { 0x72000000u, 0x34010469u },        /* at the top of the PSRAM */
    { 0x34200000u, 0x90100DA1u },        /* executing in place, in this slot */
  };

  for (unsigned i = 0; i < sizeof(good) / sizeof(good[0]); i++) {
    sim_init();
    flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
    put32(sim_flash(SLOT_A + 0x400u), good[i][0]);
    put32(sim_flash(SLOT_A + 0x404u), good[i][1]);
    CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A && !sim->entered_dfu);
    if (g_fail != 0) { printf("      vectors %08lX %08lX\n", (unsigned long)good[i][0], (unsigned long)good[i][1]); return; }
  }
}

static void t_f4_blank(void)
{
  sim_init();
  CHECK(sim_run(3) == SIM_END_DFU_IDLE && sim->entered_dfu);
}

/* F4: a slot whose vectors cannot be read is not jumped into unseen. */
static void t_f4_unread(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  sim->fail_map_call[0] = 2;                           /* after the look for a control block */
  sim->fail_map_run[0]  = 2;
  CHECK(sim_boot() == SIM_END_RESET && *sim_bkp(2) == FORCEDFU);
  sim->fail_map_call[0] = 0;
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(reset_after(3000u) == SIM_END_APP && sim->boot_offset == SLOT_A);  /* and boots when it can be */
}

static void t_f4_fsbl_in_slot(void)
{
  sim_init();
  flash_put(SLOT_A, make_fsbl(0, 84000u, 1u), 84000u);
  CHECK(sim_run(3) == SIM_END_DFU_IDLE && sim->entered_dfu);
}

static void t_f4_cut(void)
{
  uint8_t *img = make_app(2, 300000u, 78u);

  board_ab(200000u, 1u);                               /* running from B */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  app_stage_image(img, 300000u, 4u);
  sim->cut_at_op = sim->ops + 400u;                    /* the erase, some of the pages, and the power goes */
  CHECK(reset_after(2000u) == SIM_END_POWER_CUT);
  CHECK(sim_programs_in(SLOT_A, N6_OTA_SLOT_SIZE) > 300u);
  sim->cut_at_op = 0u;
  sim_power_cycle();
  *sim_flash(SLOT_B + 5000u) ^= 1u;                    /* and B has gone bad since */
  CHECK(sim_run(3) == SIM_END_DFU_IDLE && sim->entered_dfu);     /* not a jump into half an image */
}

/* F4: an update that is not an application is refused before it is written. */
static void t_f4_stage(void)
{
  uint8_t *img = make_fsbl(2, 84000u, 3u);

  board_ab(200000u, 0u);
  app_stage_image(img, 84000u, 9u);                    /* a boot loader image, staged as an update */
  sim_ops_clear();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_NOT_APP);
  CHECK(sim->ops == 0u && flash_is(SLOT_B, g_img[1], 200000u));       /* the slot to fall back on is kept */
  app_stage_image(make_app(2, 0x404u, 3u), 0x404u, 9u);               /* too short to hold its vectors */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_NOT_APP && sim->ops == 0u);
}

/* F4: a slot that matches its record is still not jumped into unless an
 * application is in it. */
static void t_f4_bad_upload(void)
{
  uint8_t *junk = make_fsbl(2, 84000u, 4u);
  n6_ota_cb_t cb;

  board_ab(200000u, 1u);                               /* running from B */
  dfu_enter();
  dfu_image(SLOT_A, junk, 84000u);                     /* not an application, uploaded to slot A */
  dfu_text("DFU_RESET");
  CHECK(sim_boot() == SIM_END_RESET);
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u);         /* recorded as the running slot, as any upload is */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u);         /* and the block names the slot that runs */
}

/* F5: the boot loader's own flash is not written without being asked. */
static void t_f5_locked(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u), *img = make_fsbl(0, 84000u, 6u);

  sim_init();
  flash_put(0u, old, 84000u);
  dfu_enter();
  dfu_image(0u, img, 84000u);
  (void)sim_boot();
  CHECK(sim_reply_has("FSBL_LOCKED") && sim_reply_has("START_BAD_ADDR"));
  CHECK(sim_erased_in(0u, SLOT_A) == 0u && sim_programs_in(0u, SLOT_A) == 0u);
  CHECK(flash_is(0u, old, 84000u));
  CHECK(sim_reply_count("N6ACK") == 0);                /* and a host that carried on was told nothing went in */
}

/* F5: the bootloader says it has the lock, so a host can tell it from one
 * that has none. */
static void t_f5_announced(void)
{
  sim_init();
  dfu_enter();
  dfu_text("DFU_INFO");
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(strstr(sim->d2h, " FSBLLOCK ") != NULL);
}

static void t_f5_unlocked(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u), *img = make_fsbl(0, 83744u, 6u);

  sim_init();
  flash_put(0u, old, 84000u);
  flash_put(0x00020000u, old, 4096u);                  /* something further up the loader's megabyte */
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(0u, img, 83744u);
  dfu_crc(0u, 83744u);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(0u, img, 83744u) && sim_reply_has("NEXT_OK") && reply_crc(sim_crc32(img, 83744u)));
  CHECK(sim->prog_dirty == 0u && sim_reply_has("FSBL_UNLOCKED"));
  CHECK(strstr(sim->d2h, "ERASE_") == NULL);           /* the erase, inside the data phase, says nothing */
  CHECK(sim_erased_in(0x15000u, SLOT_A - 0x15000u) == 0u && flash_is(0x00020000u, old, 4096u));
}

static void t_f5_wrong_image(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u), *app = make_app(0, 84000u, 6u);

  sim_init();
  flash_put(0u, old, 84000u);
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(0u, app, 84000u);                          /* an application, at the loader's address */
  (void)sim_boot();
  CHECK(sim_reply_has("DATA_NOT_FSBL"));
  CHECK(sim_erased_in(0u, SLOT_A) == 0u && sim_programs_in(0u, SLOT_A) == 0u);
  CHECK(flash_is(0u, old, 84000u));
  CHECK(!sim_reply_has("NEXT_OK"));
}

/* F5: nor one whose first block is too short to show what it is, or which
 * has no image header. */
static void t_f5_unproven(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u), *img = make_fsbl(0, 84000u, 6u);

  for (int kind = 0; kind < 2; kind++) {
    sim_init();
    flash_put(0u, old, 84000u);
    img = make_fsbl(0, 84000u, 6u);
    dfu_enter();
    /* The same bytes just went through as another image's first block; what
     * is left in the bootloader's buffers is not part of this shorter block. */
    dfu_image(0x00300000u, img, 4096u);
    if (kind == 1) { img[0] = 'X'; }
    dfu_text("DFU_FSBL_UNLOCK");
    dfu_cmd32("DFU START", 0u);
    dfu_cmd32("DFU_ERASE", 0x15000u);
    dfu_text("DFU_DATA");
    sim_host_frame(img, (kind == 0) ? 512u : 4096u);
    dfu_blocks(img + 4096u, 84000u - 4096u);
    dfu_text("DFU_NEXT");
    (void)sim_boot();
    CHECK(sim_reply_has("DATA_NOT_FSBL") && sim_reply_count("NEXT_OK") == 1);
    CHECK(sim_erased_in(0u, SLOT_A) == 0u && sim_programs_in(0u, SLOT_A) == 0u && flash_is(0u, old, 84000u));
    if (g_fail != 0) { printf("      kind %d\n", kind); return; }
  }
}

/* F5: the late erase runs silently while the host waits for the first block's
 * acknowledgement; after the whole megabyte (half a minute at the part's
 * worst) the image still lands whole. */
static void t_f5_whole_megabyte(void)
{
  uint8_t *img = make_fsbl(0, 83744u, 6u);

  sim_init();
  memset(sim_flash(0u), 0x5A, SLOT_A);
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_cmd32("DFU START", 0u);
  dfu_cmd32("DFU_ERASE", SLOT_A);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 83744u);
  dfu_text("DFU_NEXT");
  dfu_crc(0u, 83744u);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(0u, img, 83744u) && reply_crc(sim_crc32(img, 83744u)) && sim->prog_dirty == 0u);
  CHECK(flash_blank(0x15000u, SLOT_A - 0x15000u) && strstr(sim->d2h, "ERASE_") == NULL);
}

/* F5 and F11: the erase of the boot loader's flash, made late, asks the
 * flash its mode like any other. */
static void t_f5_mode_missed(void)
{
  uint8_t *img = make_fsbl(0, 83744u, 6u);

  sim_init();
  sim->bb_deaf = 1;                                    /* the start-up mode change did not take */
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(0u, img, 83744u);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(0u, img, 83744u) && sim_reply_has("NEXT_OK") && sim->prog_dirty == 0u);
}

/* F5: unlocked, the boot loader's flash is still written only from its start. */
static void t_f5_address(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u);

  sim_init();
  flash_put(0u, old, 84000u);
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(0x00001000u, make_fsbl(0, 40000u, 6u), 40000u);
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(0x000FF000u, make_fsbl(0, 4096u, 6u), 4096u);
  (void)sim_boot();
  CHECK(sim_reply_count("FSBL_LOCKED") == 2 && sim->ops == 0u && flash_is(0u, old, 84000u));
}

static void t_f5_one_shot(void)
{
  uint8_t *old = make_fsbl(1, 84000u, 5u), *img = make_fsbl(0, 84000u, 6u);

  sim_init();
  flash_put(0u, old, 84000u);
  dfu_enter();
  dfu_text("DFU_FSBL_UNLOCK");
  dfu_image(SLOT_A, make_app(2, 50000u, 7u), 50000u);  /* the unlock is spent on the next image */
  dfu_image(0u, img, 84000u);
  (void)sim_boot();
  CHECK(flash_is(0u, old, 84000u) && sim_erased_in(0u, SLOT_A) == 0u);
  CHECK(flash_is(SLOT_A, g_img[2], 50000u) && sim_reply_count("NEXT_OK") == 1);
}

/* F4: an application that executes in place has its reset vector in the
 * slot's own flash, and is an application only in the slot it was built for. */
static void t_f4_in_place(void)
{
  uint8_t *img = make_app(0, 200000u, 1u);

  put32(img + 0x404u, 0x90100DA1u);                    /* linked to run from slot A's flash */
  sim_init();
  flash_put(SLOT_A, img, 200000u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);

  board_ab(200000u, 0u);                               /* an update goes to slot B: not where it was built for */
  img = make_app(2, 200000u, 5u);
  put32(img + 0x404u, 0x90100DA1u);
  app_stage_image(img, 200000u, 6u);
  sim_ops_clear();
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_NOT_APP && sim->ops == 0u);

  board_ab(200000u, 1u);                               /* and to slot A, it is taken */
  img = make_app(2, 200000u, 5u);
  put32(img + 0x404u, 0x90100DA1u);
  app_stage_image(img, 200000u, 6u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_OK && flash_is(SLOT_A, img, 200000u));
}

/* F6: an update goes to the slot that is not running. */
static void t_f6(void)
{
  uint8_t *img = make_app(2, 300000u, 75u);
  n6_ota_cb_t cb;

  board_ab(200000u, 0u);
  *sim_flash(SLOT_A + 5000u) ^= 1u;                    /* the active slot no longer checks out */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  app_stage_image(img, 300000u, 4u);
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP);
  CHECK(sim_erased_in(SLOT_B, N6_OTA_SLOT_SIZE) == 0u);          /* the image that staged it is kept */
  CHECK(flash_is(SLOT_B, g_img[1], 200000u));
  CHECK(flash_is(SLOT_A, img, 300000u) && sim->boot_offset == SLOT_A);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == 0u);
}

/* F6: the same, when the slot that runs is slot A as the last resort. */
static void t_f6_last_resort(void)
{
  n6_ota_cb_t cb;

  board_ab(200000u, 1u);                               /* the block says B runs */
  flash_put(SLOT_A, make_app(2, 220000u, 50u), 220000u);    /* A was written by a tool that keeps no record */
  *sim_flash(SLOT_B + 5000u) ^= 1u;                    /* and B no longer checks out */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u);
  app_stage_image(make_app(0, 300000u, 76u), 300000u, 4u);
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(sim_erased_in(SLOT_A, N6_OTA_SLOT_SIZE) == 0u && flash_is(SLOT_A, g_img[2], 220000u));
  CHECK(flash_is(SLOT_B, g_img[0], 300000u));
}

/* F6: recording the slot that runs is not allowed to stop it running. */
static void t_f6_locked_flash(void)
{
  board_ab(200000u, 0u);
  *sim_flash(SLOT_A + 5000u) ^= 1u;                    /* the active slot no longer checks out */
  sim->nor_locked = 1;                                 /* and the flash takes no writes */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(sim->ops == 0u);
}

/* F7: the data phase stays inside what was erased. */
static void t_f7_no_erase(void)
{
  uint8_t *old = make_app(1, 100000u, 80u), *img = make_app(0, 100000u, 81u);

  sim_init();
  flash_put(SLOT_A, old, 100000u);
  dfu_enter();
  dfu_cmd32("DFU START", SLOT_A);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 100000u);
  dfu_text("DFU_NEXT");
  (void)sim_boot();
  CHECK(sim_reply_has("DATA_NO_ERASE"));
  CHECK(sim->programs == 0u && flash_is(SLOT_A, old, 100000u));
  CHECK(!sim_reply_has("NEXT_OK"));
  CHECK(sim_reply_count("N6ACK") == 1);                /* the DFU START's, and none after the refusal */
  CHECK(sim_reply_count("DFU_REFUSED") == 26);         /* 25 blocks and the DFU_NEXT */
}

static void t_f7_past_erase(void)
{
  uint8_t *old = make_app(1, 100000u, 80u), *img = make_app(0, 100000u, 81u);

  sim_init();
  flash_put(SLOT_A, old, 100000u);
  dfu_enter();
  dfu_cmd32("DFU START", SLOT_A);
  dfu_cmd32("DFU_ERASE", 0x1000u);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 0x3000u);                            /* three times what was erased */
  dfu_text("DFU_NEXT");
  dfu_text("DFU_INFO");
  (void)sim_boot();
  CHECK(sim_reply_has("DATA_PAST_ERASE"));
  CHECK(strstr(sim->d2h, "N6INFO") != NULL);           /* the refusal ended the data: a command is one again */
  CHECK(sim_programs_in(SLOT_A + 0x1000u, 0x100000u) == 0u && sim->prog_dirty == 0u);
  CHECK(flash_is(SLOT_A + 0x1000u, old + 0x1000u, 100000u - 0x1000u));
  CHECK(!sim_reply_has("NEXT_OK"));
  CHECK(flash_blank(SLOT_A + 0x400u, 256u));           /* what is left has no vectors, */
  CHECK(sim_run(3) == SIM_END_DFU_IDLE);               /* and is not jumped into */
}

/* F7: a block that would end exactly at the erase's end is written; one
 * byte more is not. */
static void t_f7_edge(void)
{
  uint8_t *img = make_app(0, 0x3000u, 82u);

  sim_init();
  dfu_enter();
  dfu_cmd32("DFU START", 0x00300000u);
  dfu_cmd32("DFU_ERASE", 0x2000u);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 0x2000u);
  dfu_text("DFU_NEXT");
  dfu_cmd32("DFU START", 0x00400000u);
  dfu_cmd32("DFU_ERASE", 0x2000u);
  dfu_text("DFU_DATA");
  sim_host_frame(img, 4096u);
  sim_host_frame(img + 4096u, 4000u);
  sim_host_frame(img + 8096u, 97u);                    /* one byte past the erase, with the 4000 */
  dfu_text("DFU_NEXT");
  (void)sim_boot();
  CHECK(flash_is(0x00300000u, img, 0x2000u) && sim_reply_count("NEXT_OK") == 1);
  CHECK(sim_reply_has("DATA_PAST_ERASE") && sim_programs_in(0x00402000u, 0x1000u) == 0u);
}

/* F7: a refusal lasts until the next DFU START, and an erase covers the image
 * it was made for and no other. */
static void t_f7_next_image(void)
{
  uint8_t *a = make_app(0, 60000u, 85u), *b = make_app(1, 60000u, 86u);

  sim_init();
  flash_put(0x00500000u, b, 60000u);
  dfu_enter();
  dfu_cmd32("DFU START", 0x00300000u);                 /* refused: no erase */
  dfu_text("DFU_DATA");
  dfu_blocks(a, 8192u);
  dfu_image(0x00400000u, a, 60000u);                   /* and then an image sent as it should be */
  dfu_cmd32("DFU START", 0x00500000u);                 /* and one that leans on the erase before it */
  dfu_text("DFU_DATA");
  dfu_blocks(a, 8192u);
  dfu_text("DFU_NEXT");
  (void)sim_boot();
  CHECK(flash_is(0x00400000u, a, 60000u) && sim_reply_count("NEXT_OK") == 1);
  CHECK(sim_reply_count("DATA_NO_ERASE") == 2);
  CHECK(flash_blank(0x00300000u, 0x2000u) && flash_is(0x00500000u, b, 60000u));
}

/* F7: a DFU START the loader refuses in the middle of the data ends the data. */
static void t_f7_bad_restart(void)
{
  uint8_t *img = make_app(0, 100000u, 81u);

  sim_init();
  dfu_enter();
  dfu_cmd32("DFU START", 0x00300000u);
  dfu_cmd32("DFU_ERASE", 0x19000u);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 8192u);
  dfu_cmd32("DFU START", 0x00300800u);                 /* not on a sector */
  dfu_blocks(img + 8192u, 100000u - 8192u);
  dfu_text("DFU_NEXT");
  (void)sim_boot();
  CHECK(sim_reply_has("START_BAD_ADDR") && !sim_reply_has("NEXT_OK"));
  CHECK(sim->programs == 32u && flash_blank(0x00302000u, 0x17000u));
}

/* F7 and F4: an upload to the application slot that the host abandons, or
 * that the power cuts, leaves a slot with no vectors, and the board in the
 * bootloader, waiting for the upload to be made again. */
static void t_upload_cut(void)
{
  uint8_t *img = make_app(0, 300000u, 83u);

  sim_init();
  dfu_enter();
  dfu_cmd32("DFU START", SLOT_A);
  dfu_cmd32("DFU_ERASE", 0x4A000u);
  dfu_text("DFU_DATA");
  dfu_blocks(img, 40u * 4096u);                        /* and the host goes away */
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_blank(SLOT_A + 0x400u, 256u) && flash_is(SLOT_A + 0x500u, img + 0x500u, 40u * 4096u - 0x500u));
  CHECK(reset_after(3000u) == SIM_END_RESET && *sim_bkp(2) == FORCEDFU);
  CHECK(sim_boot() == SIM_END_DFU_IDLE && sim->entered_dfu);

  sim_init();
  dfu_enter();
  dfu_image(SLOT_A, img, 300000u);
  sim->cut_at_op = 900u;                               /* the power goes in the middle of the data */
  CHECK(sim_boot() == SIM_END_POWER_CUT);
  sim->cut_at_op = 0u;
  sim_power_cycle();
  sim_host_clear();
  CHECK(sim_run(3) == SIM_END_DFU_IDLE && sim->entered_dfu);
}

/* An image shorter than the page that holds its vectors still lands whole. */
static void t_upload_small(void)
{
  static const uint32_t sizes[] = { 1200u, 0x500u, 0x501u, 4096u };

  for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
    uint8_t *img = make_app(0, sizes[i], 84u + i);

    sim_init();
    dfu_enter();
    dfu_image(SLOT_A, img, sizes[i]);
    dfu_crc(SLOT_A, sizes[i]);
    CHECK(sim_boot() == SIM_END_DFU_IDLE);
    CHECK(flash_is(SLOT_A, img, sizes[i]) && reply_crc(sim_crc32(img, sizes[i])) && sim->prog_dirty == 0u);
    if (g_fail != 0) { printf("      size %lu\n", (unsigned long)sizes[i]); return; }
  }
}

static void t_f7_wrap(void)
{
  sim_init();
  dfu_enter();
  dfu_cmd32("DFU START", SLOT_A);
  dfu_cmd32("DFU_ERASE", 0xFFF01000u);                 /* base + size wraps to one sector */
  (void)sim_boot();
  CHECK(sim_reply_has("ERASE_BAD_SIZE") && sim->erases == 0u);
}

/* F8: a settings write cut short never costs the copy that is whole. */
static void t_f8(void)
{
  static uint8_t big[900];

  for (unsigned i = 0; i < sizeof(big); i++) { big[i] = (uint8_t)(i * 7u); }
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  app_stage_asset("first settings", 14u, 1u);
  CHECK(sim_boot() == SIM_END_APP && asset_version(0) == 1u);
  app_stage_asset(big, sizeof(big), 2u);
  sim->cut_at_op = sim->ops + 2u;                      /* the erase, one page, and the power goes */
  CHECK(reset_after(2000u) == SIM_END_POWER_CUT);
  sim->cut_at_op = 0u;
  sim_power_cycle();
  CHECK(sim_boot() == SIM_END_APP);
  CHECK(asset_version(0) == 1u);                       /* the first copy is still whole */
  app_stage_asset("third settings", 14u, 3u);
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP && *sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_OK);
  CHECK(sim_erased_in(N6_ASSET_OFF(0), N6_ASSET_SECTOR) == 0u);  /* and it is not the one erased */
  CHECK(asset_version(0) == 1u && asset_version(1) == 3u);
}

/* The asset copy `c` has a header that checks out, whatever its payload. */
static int asset_header_ok(int c)
{
  n6_asset_hdr_t h;

  memcpy(&h, sim_flash(N6_ASSET_OFF(c)), sizeof(h));
  return (h.magic == N6_ASSET_MAGIC) && (h.hdr_crc == sim_crc32(&h, 16u));
}

/* F8: a copy whose payload has gone bad under a good header is not the one
 * kept. */
static void t_f8_rot(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  app_stage_asset("first settings", 14u, 1u);
  CHECK(sim_boot() == SIM_END_APP && asset_version(0) == 1u);
  app_stage_asset("second settings", 15u, 2u);
  CHECK(reset_after(2000u) == SIM_END_APP && asset_version(1) == 2u);
  *sim_flash(N6_ASSET_OFF(1) + N6_ASSET_HDR_BYTES + 3u) ^= 0x04u;      /* the newer copy loses a bit */
  CHECK(asset_header_ok(1) && asset_version(1) == 0u);
  app_stage_asset("third settings", 14u, 3u);
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP && *sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_OK);
  CHECK(sim_erased_in(N6_ASSET_OFF(0), N6_ASSET_SECTOR) == 0u);        /* the whole copy is not the one erased */
  CHECK(asset_version(0) == 1u && asset_version(1) == 3u);
}

/* F8: a write cut short at any point leaves a copy with no header. */
static void t_f8_header_last(void)
{
  static uint8_t big[900];

  for (unsigned i = 0; i < sizeof(big); i++) { big[i] = (uint8_t)(i * 11u); }
  for (uint32_t cut = 1u; cut <= 4u; cut++) {          /* the erase, then each page but the last */
    sim_init();
    flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
    app_stage_asset("first settings", 14u, 1u);
    CHECK(sim_boot() == SIM_END_APP && asset_version(0) == 1u);
    app_stage_asset(big, sizeof(big), 2u);
    sim->cut_at_op = sim->ops + cut;
    CHECK(reset_after(2000u) == SIM_END_POWER_CUT);
    CHECK(!asset_header_ok(1) && asset_version(0) == 1u);
    if (g_fail != 0) { printf("      cut after %lu operations\n", (unsigned long)cut); return; }
  }
  /* and the whole write is five operations, the header's page the last */
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  app_stage_asset(big, sizeof(big), 2u);
  CHECK(sim_boot() == SIM_END_APP && sim->ops == 5u && asset_version(0) == 2u);
  CHECK(sim->oplog[4].kind == 'P' && sim->oplog[4].addr == N6_ASSET_OFF(0));
}

/* F8: a header that claims more than a copy can hold, or does not match its
 * own check, is no copy at all, whatever version it gives. */
static void t_f8_bad_header(void)
{
  n6_asset_hdr_t h;

  for (int kind = 0; kind < 2; kind++) {
    sim_init();
    flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
    app_stage_asset("first settings", 14u, 1u);
    CHECK(sim_boot() == SIM_END_APP && asset_version(0) == 1u);
    h.magic   = N6_ASSET_MAGIC;
    h.version = 9u;
    h.len     = (kind == 0) ? 0xFFFFFFF0u : 14u;
    h.crc32   = sim_crc32("first settings", 14u);
    h.hdr_crc = sim_crc32(&h, 16u) ^ ((kind == 0) ? 0u : 1u);
    memcpy(sim_flash(N6_ASSET_OFF(1)), &h, sizeof(h));
    memcpy(sim_flash(N6_ASSET_OFF(1) + N6_ASSET_HDR_BYTES), "first settings", 14u);
    app_stage_asset("second settings", 15u, 10u);
    sim_ops_clear();
    CHECK(reset_after(2000u) == SIM_END_APP && *sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_OK);
    CHECK(sim_erased_in(N6_ASSET_OFF(0), N6_ASSET_SECTOR) == 0u);
    CHECK(asset_version(0) == 1u && asset_version(1) == 10u);
    if (g_fail != 0) { printf("      header kind %d\n", kind); return; }
  }
}

/* F8: a copy that cannot be read might be the one worth keeping, so nothing
 * is written. */
static void t_f8_unreadable(void)
{
  sim_init();
  flash_put(SLOT_A, make_app(0, 200000u, 1u), 200000u);
  app_stage_asset("first settings", 14u, 1u);
  CHECK(sim_boot() == SIM_END_APP && asset_version(0) == 1u);
  app_stage_asset("second settings", 15u, 2u);
  sim->fail_map_call[0] = 1;                           /* the look at the first copy fails every time */
  sim->fail_map_run[0]  = 2;
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP);
  CHECK(*sim_bkp(N6_ASSET_RESULT_BKPR) == N6_ASSET_RESULT_ERR_READ && sim->ops == 0u);
  CHECK(asset_version(0) == 1u && asset_version(1) == 0u);
}

/* F9: a control block that could not be read is not a board without one. */
static void t_f9_apply(void)
{
  uint8_t *img = make_app(2, 300000u, 76u);

  board_ab(200000u, 1u);                               /* running from B */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  app_stage_image(img, 300000u, 4u);
  sim->fail_map_call[0] = 1;                           /* the control block read fails once */
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP);
  CHECK(sim_erased_in(SLOT_B, N6_OTA_SLOT_SIZE) == 0u && flash_is(SLOT_B, g_img[1], 200000u));
  CHECK(flash_is(SLOT_A, img, 300000u) && sim->boot_offset == SLOT_A);     /* and the update went where it should */
}

static void t_f9_apply_hard(void)
{
  uint8_t *img = make_app(2, 300000u, 76u);

  board_ab(200000u, 1u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  app_stage_image(img, 300000u, 4u);
  sim->fail_map_call[0] = 1;                           /* the flash cannot be read at all */
  sim->fail_map_run[0]  = 1000;
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_RESET && *sim_bkp(2) == FORCEDFU);
  CHECK(sim->ops == 0u && *sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_FLASH_READ);
  sim->fail_map_call[0] = 0;
  CHECK(sim_boot() == SIM_END_DFU_IDLE && sim->entered_dfu);
  CHECK(reset_after(3000u) == SIM_END_APP && sim->boot_offset == SLOT_B);  /* the board is as it was */

  app_stage_image(img, 300000u, 4u);
  sim->fail_map_call[0] = 1;                           /* every try at the control block fails, */
  sim->fail_map_run[0]  = 2;                           /* and the flash reads again after it */
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(sim->ops == 0u && *sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_FLASH_READ);
  CHECK(flash_is(SLOT_B, g_img[1], 200000u) && flash_is(SLOT_A, g_img[0], 200000u));
}

/* F9: the control block read back after the apply's first write fails. The
 * slot has not been touched, and the update goes on with what was written. */
static void t_f9_reload(void)
{
  uint8_t *img = make_app(2, 300000u, 76u);
  n6_ota_cb_t cb;

  board_ab(200000u, 1u);
  app_stage_image(img, 300000u, 4u);
  sim->fail_map_call[0] = 2;                           /* the first read works, the read back does not */
  sim->fail_map_run[0]  = 2;
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_OK && flash_is(SLOT_A, img, 300000u));
  CHECK(flash_is(SLOT_B, g_img[1], 200000u));
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == 0u && cb.slot[1].state == N6_OTA_SLOT_CONFIRMED);
}

static void t_f9_boot(void)
{
  board_ab(200000u, 1u);
  sim->fail_map_call[0] = 1;
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(sim->ops == 0u);
}

static void t_f9_boot_hard(void)
{
  board_ab(200000u, 1u);
  sim->fail_map_call[0] = 1;                           /* every try at the control block fails, */
  sim->fail_map_run[0]  = 2;                           /* and the flash reads again after it */
  CHECK(sim_boot() == SIM_END_RESET && *sim_bkp(2) == FORCEDFU);       /* the bootloader, not slot A */
  CHECK(sim->ops == 0u);
}

/* F9: a confirmation the loader could not act on is kept for the next boot. */
static void t_f9_confirm_kept(void)
{
  uint8_t *img = make_app(2, 300000u, 79u);
  n6_ota_cb_t cb;

  board_ab(200000u, 0u);
  app_stage_image(img, 300000u, 3u);
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_B);      /* on trial */
  app_confirm();
  sim->fail_map_call[0] = 1;
  sim->fail_map_run[0]  = 1000;
  CHECK(reset_after(5000u) == SIM_END_RESET);
  CHECK(*sim_bkp(N6_OTA_CONFIRM_BKPR) == N6_OTA_CONFIRM_MAGIC);
  sim->fail_map_call[0] = 0;
  CHECK(sim_run(2) == SIM_END_DFU_IDLE);
  CHECK(reset_after(3000u) == SIM_END_APP && sim->boot_offset == SLOT_B);
  CHECK(cb_read(&cb) >= 0 && cb.active == 1u && cb.pending == N6_OTA_NO_PENDING);
}

/* F9: a confirmation the commit could not spend belongs to the running image,
 * so an update staged beside it is not applied on that boot (it would inherit
 * the confirmation and lose its rollback). */
static void t_f9_confirm_not_inherited(void)
{
  uint8_t *img = make_app(2, 300000u, 90u);
  n6_ota_cb_t cb;

  board_ab(200000u, 0u);                               /* A runs, both slots confirmed */
  CHECK(sim_boot() == SIM_END_APP && sim->boot_offset == SLOT_A);
  app_confirm();                                       /* A confirms, as it does every boot */
  app_stage_image(img, 300000u, 3u);                   /* and stages an update */
  sim->fail_map_call[0] = 1;                           /* the commit's read fails, every try */
  sim->fail_map_run[0]  = 2;
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A && !sim->wdg_running);
  sim->fail_map_call[0] = 0;
  sim->fail_map_run[0]  = 0;
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_FLASH_READ);
  CHECK(flash_is(SLOT_B, g_img[1], 200000u));           /* the update was not written */
  CHECK(*sim_bkp(N6_OTA_CONFIRM_BKPR) == N6_OTA_CONFIRM_MAGIC);
  /* The next boot spends it on A, whose it is. */
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A && !sim->wdg_running);
  CHECK(*sim_bkp(N6_OTA_CONFIRM_BKPR) == 0u);
  CHECK(cb_read(&cb) >= 0 && cb.active == 0u && cb.pending == N6_OTA_NO_PENDING);
  CHECK(flash_is(SLOT_B, g_img[1], 200000u));
}

/* F9: an upload the control block could not record tells the host. */
static void t_f9_upload(void)
{
  uint8_t *img = make_app(2, 180000u, 31u);

  board_ab(200000u, 1u);
  dfu_enter();
  dfu_image(SLOT_A, img, 180000u);
  sim->fail_map_call[0] = 2;                           /* after the bootloader's look for a trial */
  sim->fail_map_run[0]  = 2;
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(sim_reply_has("OTA_CB_NOT_UPDATED") && !sim_reply_has("OTA_CB_UPDATED") && sim_reply_has("NEXT_OK"));
}

/* The first update, with slot A's check read failing. */
static void t_first_gap(void)
{
  uint8_t *img = make_app(2, 350000u, 77u);
  int end;

  sim_init();
  flash_put(SLOT_A, make_app(0, 250000u, 1u), 250000u);
  CHECK(sim_boot() == SIM_END_APP);
  app_stage_image(img, 350000u, 2u);
  sim->fail_map_call[0] = 2;                           /* the read of slot A, after the control block's */
  end = reset_after(2000u);
  sim->fail_map_call[0] = 0;
  for (int i = 0; (i < 6) && (end == SIM_END_APP) && (sim->boot_offset == SLOT_B); i++) {
    end = reset_after(2000u);                          /* never confirmed */
  }
  CHECK(end == SIM_END_APP && sim->boot_offset == SLOT_A);
}

/* The same, with slot A unreadable every time: no record of it, so no update. */
static void t_first_gap_hard(void)
{
  uint8_t *img = make_app(2, 350000u, 77u);

  sim_init();
  flash_put(SLOT_A, make_app(0, 250000u, 1u), 250000u);
  CHECK(sim_boot() == SIM_END_APP);
  app_stage_image(img, 350000u, 2u);
  sim->fail_map_call[0] = 2;                           /* the control block's read works; slot A's never does */
  sim->fail_map_run[0]  = 2;
  sim_ops_clear();
  CHECK(reset_after(2000u) == SIM_END_APP && sim->boot_offset == SLOT_A);
  CHECK(*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_ERR_FLASH_READ);
  CHECK(sim->ops == 0u && flash_blank(N6_OTA_CB_OFF(0), 0x2000u));
}

/* F11: an erase sends the flash nothing it cannot hear. */
static void t_f11(void)
{
  const uint32_t size = 100000u;
  uint8_t *img = make_app(0, size, 61u);

  sim_init();
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(sim->spi_in_opi == 0u && flash_is(SLOT_A, img, size));
}

static void t_f11_floating_low(void)
{
  const uint32_t size = 100000u;
  uint8_t *img = make_app(0, size, 62u);

  sim_init();
  sim->io1_high = 0;                                   /* the undriven line reads low on this board */
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(SLOT_A, img, size) && sim_reply_has("NEXT_OK"));
}

/* F11: a flash the start-up mode change did not reach is changed at the
 * erase, whichever way the undriven line floats. */
static void t_f11_mode_missed_low(void)
{
  const uint32_t size = 100000u;
  uint8_t *img = make_app(0, size, 63u);

  sim_init();
  sim->bb_deaf  = 1;
  sim->io1_high = 0;
  dfu_enter();
  dfu_image(SLOT_A, img, size);
  CHECK(sim_boot() == SIM_END_DFU_IDLE);
  CHECK(flash_is(SLOT_A, img, size) && sim_reply_has("NEXT_OK") && sim->spi_in_opi == 0u);
}

/* F11: a flash that will not answer is an erase refused, with the bootloader
 * still there to say so. */
static void t_f11_no_answer(void)
{
  for (int high = 0; high < 2; high++) {
    sim_init();
    sim->nor_locked = 1;                               /* it takes no write enable, in either mode */
    sim->io1_high   = high;
    dfu_enter();
    dfu_image(SLOT_A, make_app(0, 100000u, 64u), 100000u);
    dfu_text("DFU_INFO");
    CHECK(sim_boot() == SIM_END_DFU_IDLE);
    CHECK(sim_reply_has("ERASE_FLASH_MODE") && sim->ops == 0u && !sim_reply_has("NEXT_OK"));
    CHECK(strstr(sim->d2h, "N6INFO") != NULL);         /* and it still answers a host */
    if (g_fail != 0) { printf("      undriven line %s\n", high ? "high" : "low"); return; }
  }
}

/* The list. */

struct test {
  const char *id;
  const char *what;
  void (*fn)(void);
  int open;                 /* an open finding: the test states what must hold once it is fixed */
};

static const struct test TESTS[] = {
  { "boot",     "a board with no control block boots slot A, and writes nothing", t_boot_legacy, 0 },
  { "tap",      "a second reset within a second enters the bootloader", t_double_tap, 0 },
  { "dfu",      "the forced-bootloader key enters it once, and it identifies itself", t_forced_dfu, 0 },
  { "upload",   "an application uploaded over USB lands whole and boots", t_dfu_upload, 0 },
  { "upload2",  "weights and an application in one session", t_dfu_weights_and_app, 0 },
  { "refuse",   "a bad address or an erase out of its region is refused", t_dfu_bad_requests, 0 },
  { "note",     "an upload to a slot on an updated board becomes the running slot", t_dfu_notes_slot, 0 },
  { "cancel",   "an upload into a slot cancels the update and settings the application staged", t_dfu_cancels_staged, 0 },
  { "cancel2",  "the same on an updated board, where the upload is also recorded", t_dfu_cancels_staged_ab, 0 },
  { "keep",     "an upload of weights alone leaves what the application staged armed", t_dfu_weights_keep_staged, 0 },
  { "update",   "a staged update is applied, tried, confirmed and committed", t_update_confirm, 0 },
  { "rollback", "an update never confirmed is given up after three tries", t_update_rolls_back, 0 },
  { "stage",    "a staged image that fails its CRC is refused, flash untouched", t_update_refuses_bad_stage, 0 },
  { "first",    "a board's first update can roll back", t_first_update, 0 },
  { "asset",    "stored settings alternate between the two copies", t_asset, 0 },
  { "overlap",  "a full-size image staged with settings is refused, flash untouched", t_stage_overlap, 0 },
  { "stale",    "slot A written by a tool that keeps no record still boots", t_stale_record_still_boots, 0 },
  { "modesw",   "an upload works when the start-up mode change did not take", t_mode_switch_missed, 0 },
  { "f2host",   "a host that speaks keeps the bootloader during a trial", t_f2_host, 0 },
  { "f4",       "F4  a slot whose vectors make no sense is not jumped into", t_f4_garbage, 1 },
  { "f4good",   "F4  and every stack and reset vector an application can have is accepted", t_f4_accepts, 0 },
  { "f4blank",  "F4  a blank board goes to the bootloader", t_f4_blank, 1 },
  { "f4unread", "F4  a slot whose vectors cannot be read is not jumped into", t_f4_unread, 1 },
  { "f4fsbl",   "F4  a boot loader image in an application slot is not jumped into", t_f4_fsbl_in_slot, 1 },
  { "f4cut",    "F4  a slot an update left half written is not jumped into", t_f4_cut, 1 },
  { "f4stage",  "F4  a staged update that is not an application is refused, flash untouched", t_f4_stage, 1 },
  { "f4upload", "F4  a slot that matches its record but holds no application is passed over", t_f4_bad_upload, 1 },
  { "f4xip",    "F4  an application that executes in place boots, from the slot it was built for", t_f4_in_place, 1 },
  { "f6",       "F6  an update never overwrites the image that staged it", t_f6, 1 },
  { "f6last",   "F6  nor when that image runs from slot A as the last resort", t_f6_last_resort, 1 },
  { "f6locked", "F6  a flash that takes no writes does not stop the slot that checks out from booting", t_f6_locked_flash, 0 },
  { "f9",       "F9  a control block read that fails once does not send an update to the running slot", t_f9_apply, 1 },
  { "f9hard",   "F9  with the control block unreadable, an update is refused, flash untouched", t_f9_apply_hard, 1 },
  { "f9reload", "F9  a control block that cannot be read back mid-update does not stop the update", t_f9_reload, 1 },
  { "f9boot",   "F9  a control block read that fails once does not boot the wrong slot", t_f9_boot, 1 },
  { "f9dfu",    "F9  with the control block unreadable, the board goes to the bootloader", t_f9_boot_hard, 1 },
  { "f9keep",   "F9  a confirmation that could not be acted on is kept", t_f9_confirm_kept, 1 },
  { "f9stale",  "F9  a kept confirmation is not inherited by an update staged beside it", t_f9_confirm_not_inherited, 1 },
  { "f9upload", "F9  an upload the control block could not record says so", t_f9_upload, 1 },
  { "gap",      "     a first update whose slot A check fails to read once can still roll back", t_first_gap, 1 },
  { "gaphard",  "     and with slot A unreadable there is no update", t_first_gap_hard, 1 },
  { "f1",       "F1  a working update survives a board only ever switched off and on", t_f1, 1 },
  { "f1mixed",  "F1  a boot after a power cut neither uses a try nor gives one back", t_f1_mixed, 1 },
  { "f1regs",   "F1  an application's own backup registers do not hide its resets", t_f1_app_registers, 0 },
  { "f2",       "F2  a trial image that faults is rolled back with nobody there", t_f2, 1 },
  { "f2tap",    "F2  and so is one that resets within a second of starting", t_f2_tap, 1 },
  { "f2stay",   "F2  with no update on trial the bootloader waits as long as it takes", t_f2_stay, 0 },
  { "f3",       "F3  a trial image that hangs is reset by a watchdog and rolled back", t_f3, 1 },
  { "f3apply",  "F3  the watchdog covers the longest apply without cutting it short", t_f3_apply, 1 },
  { "f3slow",   "F3  nor a full-size apply on a flash that programs eight times slower", t_f3_slow, 0 },
  { "f3late",   "F3  a trial image that confirms without a watchdog is reset once, and committed", t_f3_confirms, 1 },
  { "f3spin",   "F3  a flash write that hangs in the apply ends in a reset, and the old image boots", t_f3_spin, 1 },
  { "f3crc",    "F3  an update that fails its check is not followed by the old image under a watchdog", t_f3_bad_write, 1 },
  { "f5",       "F5  the boot loader's flash is not written without the unlock", t_f5_locked, 1 },
  { "f5info",   "F5  the bootloader says it has the lock", t_f5_announced, 1 },
  { "f5ok",     "F5  unlocked, a boot loader image is written and checks out", t_f5_unlocked, 1 },
  { "f5wrong",  "F5  unlocked, an image that is not a boot loader erases nothing", t_f5_wrong_image, 1 },
  { "f5proof",  "F5  nor one too short to tell, or with no image header", t_f5_unproven, 1 },
  { "f5big",    "F5  the late erase of the loader's whole megabyte, in silence, and the image whole after it", t_f5_whole_megabyte, 1 },
  { "f5mode",   "F5  the late erase of the boot loader's flash asks the flash its mode first", t_f5_mode_missed, 0 },
  { "f5addr",   "F5  unlocked, the boot loader's flash is written from its start only", t_f5_address, 1 },
  { "f5once",   "F5  the unlock covers the next image only", t_f5_one_shot, 1 },
  { "f7",       "F7  data is refused without an erase, and nothing after it is acknowledged", t_f7_no_erase, 1 },
  { "f7past",   "F7  data past the erased range is not written", t_f7_past_erase, 1 },
  { "f7edge",   "F7  data may fill the erased range to its last byte, and not one more", t_f7_edge, 1 },
  { "f7next",   "F7  a refusal ends at the next DFU START, and an erase serves one image only", t_f7_next_image, 1 },
  { "f7start",  "F7  a DFU START refused in the middle of the data ends the data", t_f7_bad_restart, 1 },
  { "f7wrap",   "F7  an erase size that wraps the address is refused", t_f7_wrap, 1 },
  { "cut",      "F4  an upload cut short leaves the board in the bootloader, not in half an image", t_upload_cut, 1 },
  { "small",    "     an image that ends in or just past the page of its vectors lands whole", t_upload_small, 0 },
  { "f8",       "F8  a settings write cut short never costs the whole copy", t_f8, 1 },
  { "f8rot",    "F8  nor does a copy whose payload has gone bad under a good header", t_f8_rot, 1 },
  { "f8last",   "F8  a settings write cut short at any point leaves a copy with no header", t_f8_header_last, 1 },
  { "f8hdr",    "F8  a header that fails its check or claims too much is no copy at all", t_f8_bad_header, 1 },
  { "f8read",   "F8  with a stored copy unreadable, settings are not written", t_f8_unreadable, 1 },
  { "f11",      "F11 an erase sends an octal flash no single-line commands", t_f11, 1 },
  { "f11low",   "F11 an erase works whichever way the undriven line floats", t_f11_floating_low, 1 },
  { "f11miss",  "F11 a flash the start-up mode change missed is changed at the erase, either way", t_f11_mode_missed_low, 1 },
  { "f11dead",  "F11 a flash that does not answer is an erase refused, the bootloader still there", t_f11_no_answer, 1 },
};

int main(int argc, char **argv)
{
  int failed = 0, open = 0, passed = 0;

  for (unsigned i = 0; i < sizeof(TESTS) / sizeof(TESTS[0]); i++) {
    const struct test *t = &TESTS[i];

    if ((argc > 1) && (strcmp(argv[1], t->id) != 0)) {
      continue;
    }
    g_fail = 0;
    t->fn();
    if (t->open) {
      if (g_fail != 0) {
        printf("OPEN  %-9s %s\n", t->id, t->what);
        open++;
      } else {
        printf("FAIL  %-9s %s: passes, but is listed as an open finding\n", t->id, t->what);
        failed++;
      }
    } else if (g_fail != 0) {
      printf("FAIL  %-9s %s\n", t->id, t->what);
      failed++;
    } else {
      printf("PASS  %-9s %s\n", t->id, t->what);
      passed++;
    }
  }
  printf("%d passed, %d failed, %d open findings reproduced\n", passed, failed, open);
  return failed ? 1 : 0;
}
