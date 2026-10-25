/* The boot loader's source on the simulated board, driven at random.
 *
 * test_fsbl.c holds directed cases; this generates thousands from a seed and
 * holds the loader to what must be true of all of them:
 *
 *   uploads  A host sends requests in any order, sensible or not. A small
 *            model of the upload protocol says what the flash may then hold.
 *            The loader must not erase or program outside what an accepted
 *            erase covered, must never touch its own flash unless that was
 *            unlocked and the image is a boot loader's, must leave every
 *            image it accepted whole, and must still be answering.
 *
 *   boots    A board in any state: with or without a control block, slots
 *            whole, blank, damaged or holding the wrong thing, an update or
 *            settings staged, a confirmation set or not. Then resets, power
 *            cuts in the middle of flash writes, reads that fail, requests
 *            for the bootloader. Whatever the loader jumps into must be a
 *            whole application; it must never write over the image it last
 *            ran outside a trial, nor erase a slot for an update that is
 *            damaged or is not an application, nor the only whole copy of
 *            the settings; the watchdog must be running for an image on
 *            trial and for no other; an update on trial must not be left in
 *            the bootloader; and left alone the board must settle.
 *
 * It is the weaker of the two: of the 80 changes test_fsbl.c is known to
 * catch, 1500 runs of this catch about half.
 *
 *   sh harness/bench/hosttest/fsbl/run.sh fuzz [runs [seed [run [uploads|boots]]]]
 *
 * A failure prints the seed and run that made it: the same two numbers make
 * it again, and a run number given third makes that run alone. "uploads" or
 * "boots" makes every run one of that kind.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "n6_ota_layout.h"
#include "fsbl_sim.h"

#define FORCEDFU  0x00F4CDF4u
#define SLOT_A    N6_OTA_SLOT_A_OFF
#define SLOT_B    N6_OTA_SLOT_B_OFF
#define FLASH_MAX SIM_FLASH_SIZE

/* Random numbers. */

static uint32_t g_x;

static uint32_t rnd(void)
{
  g_x ^= g_x << 13;
  g_x ^= g_x >> 17;
  g_x ^= g_x << 5;
  return g_x;
}

static uint32_t below(uint32_t n) { return (n == 0u) ? 0u : (rnd() % n); }
static int      chance(uint32_t percent) { return below(100u) < percent; }

/* Report. */

/* What the runs amounted to, so that "every one held" says of what. */
static struct {
  unsigned long images, loaders, refusals;                       /* uploads */
  unsigned long entered, trials, applied, cuts, bootloader;      /* boots */
} g_seen;

static int         g_bad;
static const char *g_what = "";
static uint32_t    g_seed, g_run;
static char        g_story[16384];

static void story(const char *fmt, unsigned long a, unsigned long b)
{
  const size_t n = strlen(g_story);

  if (n + 96u < sizeof(g_story)) {
    snprintf(g_story + n, sizeof(g_story) - n, fmt, a, b);
  }
}

static void fail(const char *why, unsigned long a, unsigned long b)
{
  if (g_bad++ == 0) {
    printf("FAIL  %s, seed %lu run %lu: ", g_what, (unsigned long)g_seed, (unsigned long)g_run);
    printf(why, a, b);
    printf("\n      %s\n", g_story);
  }
}

/* Images. */

static uint8_t g_img[4][N6_OTA_SLOT_SIZE];

static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4u); }

static uint8_t *make_image(int which, uint32_t size, int fsbl)
{
  uint8_t *img = g_img[which];
  uint32_t x = rnd() | 1u;

  for (uint32_t i = 0u; i < size; i += 4u) {
    x = x * 1664525u + 1013904223u;
    memcpy(img + i, &x, 4u);
  }
  memcpy(img, "STM2", 4u);
  put32(img + 0x400u, 0x34200000u);
  put32(img + 0x404u, (fsbl ? 0x34180401u : 0x34000401u) + (below(0x4000u) << 1));
  return img;
}

static int frame_is_fsbl(const uint8_t *p, uint32_t len)
{
  uint32_t sp, pc;

  if ((len < 0x408u) || (memcmp(p, "STM2", 4u) != 0)) { return 0; }
  memcpy(&sp, p + 0x400u, 4u);
  memcpy(&pc, p + 0x404u, 4u);
  return ((sp & 3u) == 0u) && (sp > 0x20000000u) && (sp <= 0x40000000u) &&
         ((pc & 1u) != 0u) && (pc >= 0x34180400u) && (pc < 0x34200000u);
}

/* Uploads. */

/* What the upload protocol says, written without looking at the loader. */
static struct {
  int      unlocked, started, data, refusing, fsbl, deferred;
  uint32_t base, erased, wrote;
  /* the ranges an accepted erase covered, and so the only flash that may change */
  struct { uint32_t off, len; } may[64];
  int      mays;
  /* the images the loader accepted to the end */
  struct { uint32_t off, len; const uint8_t *data; int mays; } whole[16];
  int      wholes;
  int      reset;                /* the loader has reset on DFU_END */
  const uint8_t *cur;            /* the bytes of the image being sent */
} m;

static void m_may(uint32_t off, uint32_t len);

static void m_refuse(void) { m.started = m.data = m.deferred = 0; m.refusing = 1; g_seen.refusals++; }

/* A frame that arrives in the data phase and is neither of its two requests
 * is data, whatever it says. */
static void m_taken_as_data(const uint8_t *p, uint32_t len)
{
  if (m.refusing) { return; }
  if (m.wrote + len > m.erased) { m_refuse(); return; }
  if (m.deferred) {
    if (!frame_is_fsbl(p, len)) { m_refuse(); return; }
    m_may(m.base, m.erased);
    m.deferred = 0;
  }
  m.wrote += len;
}

static uint32_t region_end(uint32_t base)
{
  if (base < SLOT_A)                       { return SLOT_A; }
  if (base < N6_OTA_CB_OFF(0))             { return N6_OTA_CB_OFF(0); }
  if (base < 0x00300000u)                  { return 0x00300000u; }
  if (base < SLOT_B)                       { return SLOT_B; }
  if (base < SLOT_B + N6_OTA_SLOT_SIZE)    { return SLOT_B + N6_OTA_SLOT_SIZE; }
  return FLASH_MAX;
}

static void m_may(uint32_t off, uint32_t len)
{
  if (m.mays < 64) { m.may[m.mays].off = off; m.may[m.mays].len = len; m.mays++; }
}

static void send32(const char *name, uint32_t v)
{
  uint8_t f[16];
  const uint32_t n = (uint32_t)strlen(name);

  memcpy(f, name, n);
  memcpy(f + n, &v, 4u);
  sim_host_frame(f, n + 4u);
}

static void do_unlock(void)
{
  sim_host_frame("DFU_FSBL_UNLOCK", 15u);
  story(" unlock", 0, 0);
  if (!m.data) { m.unlocked = 1; }
  else         { m_taken_as_data((const uint8_t *)"DFU_FSBL_UNLOCK", 15u); m.cur = NULL; }
}

static void do_start(uint32_t addr)
{
  const int unlocked = m.unlocked;

  send32("DFU START", addr);
  story(" start %lX", addr, 0);
  m.unlocked = 0;
  if (((addr % 0x1000u) != 0u) || (addr >= FLASH_MAX))      { m_refuse(); return; }
  if ((addr < SLOT_A) && !(unlocked && (addr == 0u)))       { m_refuse(); return; }
  m.started = 1; m.data = 0; m.refusing = 0; m.deferred = 0;
  m.base = addr; m.erased = 0u; m.wrote = 0u; m.fsbl = (addr < SLOT_A); m.cur = NULL;
}

static void do_erase(uint32_t size)
{
  send32("DFU_ERASE", size);
  story(" erase %lX", size, 0);
  if (m.data) {
    uint8_t f[13];

    memcpy(f, "DFU_ERASE", 9u);
    memcpy(f + 9, &size, 4u);
    m_taken_as_data(f, 13u);
    m.cur = NULL;
    return;
  }
  if (m.refusing || !m.started) { return; }
  if ((size == 0u) || ((size % 0x1000u) != 0u) || (size > region_end(m.base) - m.base)) { m_refuse(); return; }
  m.erased = size;
  if (m.fsbl) { m.deferred = 1; } else { m_may(m.base, size); }
}

static void do_data(void)
{
  sim_host_frame("DFU_DATA", 8u);
  story(" data", 0, 0);
  if (m.data) { m_taken_as_data((const uint8_t *)"DFU_DATA", 8u); m.cur = NULL; return; }
  if (m.refusing || !m.started) { return; }
  if (m.erased == 0u) { m_refuse(); return; }
  m.data = 1; m.started = 0;
}

static void do_block(const uint8_t *p, uint32_t len)
{
  sim_host_frame(p, len);
  story(" block %lu", len, 0);
  if (m.data) { m_taken_as_data(p, len); }
}

/* A request the bootloader answers without anything changing, unless it
 * arrives in the data phase. */
static void do_other(const char *text)
{
  sim_host_frame(text, (uint32_t)strlen(text));
  story(" other", 0, 0);
  if (m.data) { m_taken_as_data((const uint8_t *)text, (uint32_t)strlen(text)); m.cur = NULL; }
}

static void do_finish(int end)
{
  if (end) { sim_host_frame("DFU_END", 7u); } else { sim_host_frame("DFU_NEXT", 8u); }
  story(end ? " end" : " next", 0, 0);
  if (!m.data || m.refusing) { return; }
  if ((m.cur != NULL) && (m.wrote > 0u) && (m.wholes < 16)) {
    m.whole[m.wholes].off  = m.base;
    m.whole[m.wholes].len  = m.wrote;
    m.whole[m.wholes].data = m.cur;
    m.whole[m.wholes].mays = m.mays;
    m.wholes++;
    g_seen.images++;
    if (m.base == 0u) { g_seen.loaders++; }
  }
  m.data = 0; m.started = 0; m.deferred = 0;
  m.reset = end;                              /* DFU_END resets the board: nothing after it is heard */
}

static uint32_t some_address(void)
{
  static const uint32_t pick[] = {
    0u, 0x1000u, 0x00014000u, SLOT_A - 0x1000u, SLOT_A, SLOT_A + 0x1000u, SLOT_A + N6_OTA_SLOT_SIZE - 0x1000u,
    N6_OTA_CB_OFF(0), N6_OTA_CB_OFF(1), N6_ASSET_OFF(0), 0x00300000u, 0x00400000u, SLOT_B - 0x1000u, SLOT_B,
    SLOT_B + N6_OTA_SLOT_SIZE - 0x2000u, 0x01E00000u, FLASH_MAX - 0x1000u, FLASH_MAX, 0xFFFFF000u, 0x00100800u,
  };

  return chance(85) ? pick[below(sizeof(pick) / sizeof(pick[0]))] : (below(FLASH_MAX / 0x1000u) * 0x1000u);
}

static uint32_t some_size(uint32_t want)
{
  static const uint32_t pick[] = { 0u, 0x1000u, 0x2000u, 0x10000u, 0x11000u, 0x800u, 0x00180000u, 0x00181000u,
                                   0xFFF01000u, 0xFFFFF000u, 0x02000000u };

  if (chance(60)) { return (want + 0xFFFu) & ~0xFFFu; }
  return pick[below(sizeof(pick) / sizeof(pick[0]))];
}

/* One image, sent more or less as it should be. */
static void send_image(int slot)
{
  /* Three uploads in ten aim at the boot loader's own flash: most as a proper
   * update of it, the rest by accident. */
  const int at_loader = chance(30);
  const int fsbl = at_loader ? chance(75) : chance(10);
  /* One size in five is a whole number of sectors, so the image fills what
   * is erased for it to the last byte. */
  const uint32_t some = 1u + below(chance(80) ? 40000u : 300000u);
  const uint32_t size = chance(20) ? ((some + 0xFFFu) & ~0xFFFu) : some;
  uint8_t *img = make_image(slot, (size + 3u) & ~3u, fsbl);
  uint32_t sent = 0u;

  if (at_loader ? chance(80) : chance(10)) { do_unlock(); }
  do_start((at_loader && chance(80)) ? 0u : some_address());
  if (chance(90)) { do_erase(some_size(size)); }
  if (chance(8))  { do_unlock(); }
  if (chance(92)) { do_data(); }
  m.cur = img;
  while (sent < size) {
    uint32_t n = chance(85) ? 4096u : (1u + below(4096u));

    if (n > size - sent) { n = size - sent; }
    /* a block the loader would take for a request is sent as one, below */
    if ((n == 7u) || (n == 8u) || (n == 13u) || (n == 15u)) { n = (size - sent > 16u) ? 16u : n; }
    if ((n == 7u) || (n == 8u) || (n == 13u) || (n == 15u)) { break; }
    do_block(img + sent, n);
    sent += n;
    if (chance(2)) { do_start(some_address()); }          /* a host that starts over in the middle */
    if (chance(1)) { do_other("DFU_INFO"); }
  }
  if (m.data && (m.wrote != sent)) { m.cur = NULL; }       /* not this image from its first byte */
  if (chance(92)) { do_finish(chance(15)); }
}

static void fuzz_upload(void)
{
  const int images = 1 + (int)below(3u);

  g_what = "uploads";
  g_story[0] = '\0';
  memset(&m, 0, sizeof(m));
  sim_init();
  memset(sim_flash(0u), 0x5A, FLASH_MAX);            /* nothing erased: a write without an erase shows */
  sim->io1_high = (int)below(2u);
  *sim_bkp(2) = FORCEDFU;
  sim_host_clear();

  for (int i = 0; (i < images) && !m.reset; i++) {
    send_image(i);
    if (!m.reset && chance(10)) { do_other("DFU_INFO"); }
    if (!m.reset && chance(4))  { do_other("WHAT IS THIS"); }
  }

  {
    const int end = sim_boot();

    if (end != (m.reset ? SIM_END_RESET : SIM_END_DFU_IDLE)) {
      fail("the bootloader ended as %lu, not %lu", (unsigned long)end,
           (unsigned long)(m.reset ? SIM_END_RESET : SIM_END_DFU_IDLE));
    }
  }
  if ((sim->prog_dirty != 0u) || (sim->no_wel != 0u)) {
    fail("%lu bytes programmed that were not erased, %lu writes without write enable", sim->prog_dirty, sim->no_wel);
  }
  /* every erase and program inside what an accepted erase covered */
  for (uint32_t i = 0u; (i < sim->ops) && (i < SIM_OPLOG); i++) {
    const uint32_t a = sim->oplog[i].addr, l = sim->oplog[i].len;
    int ok = 0;

    for (int k = 0; (k < m.mays) && !ok; k++) {
      ok = (a >= m.may[k].off) && (a + l <= m.may[k].off + m.may[k].len);
    }
    if (!ok) { fail("flash at %lX, %lu bytes, was written outside any accepted erase", a, l); break; }
  }
  /* every image accepted to its end is there, unless a later erase took it */
  for (int k = 0; k < m.wholes; k++) {
    int later = 0;

    for (int j = m.whole[k].mays; j < m.mays; j++) {
      later |= (m.may[j].off < m.whole[k].off + m.whole[k].len) && (m.may[j].off + m.may[j].len > m.whole[k].off);
    }
    if (!later && (memcmp(sim_flash(m.whole[k].off), m.whole[k].data, m.whole[k].len) != 0)) {
      uint32_t at = 0u;

      while (*sim_flash(m.whole[k].off + at) == m.whole[k].data[at]) { at++; }
      fail("the image accepted at %lX is not what was sent, from its byte %lu on", m.whole[k].off, at);
    }
  }
}

/* Boots. */

/* The whole application images this board has been given: what a slot may
 * hold when the loader jumps into it. */
static struct {
  uint8_t *img;
  uint32_t size;
} g_known[16];
static int g_knowns;

static void know(const uint8_t *img, uint32_t size)
{
  if (g_knowns < 16) {
    g_known[g_knowns].img = malloc(size);
    memcpy(g_known[g_knowns].img, img, size);
    g_known[g_knowns].size = size;
    g_knowns++;
  }
}

static void forget_all(void)
{
  while (g_knowns > 0) { free(g_known[--g_knowns].img); }
}

static int slot_is_whole(uint32_t off)
{
  for (int k = 0; k < g_knowns; k++) {
    if (memcmp(sim_flash(off), g_known[k].img, g_known[k].size) == 0) { return 1; }
  }
  return 0;
}

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

/* Put something in a slot, and say what the control block should record of it. */
static void fill_slot(int slot, n6_ota_cb_t *cb)
{
  const uint32_t off  = slot ? SLOT_B : SLOT_A;
  const uint32_t size = 0x1000u + (below(200000u) & ~3u);
  uint8_t *img = make_image(slot, size, 0);
  const uint32_t kind = below(100u);

  memcpy(sim_flash(off), img, size);
  cb->slot[slot].crc32 = sim_crc32(img, size);
  cb->slot[slot].size  = size;
  cb->slot[slot].state = chance(70) ? N6_OTA_SLOT_CONFIRMED : N6_OTA_SLOT_VALID;
  know(img, size);

  if (kind < 55u) {
    story(" %lu:whole", (unsigned long)slot, 0);
  } else if (kind < 65u) {                    /* blank */
    memset(sim_flash(off), 0xFF, size);
    if (chance(50)) { cb->slot[slot].state = N6_OTA_SLOT_EMPTY; }
    story(" %lu:blank", (unsigned long)slot, 0);
  } else if ((kind < 75u) && (slot == 1)) {   /* a bit gone bad, past the vectors. Slot B only: in slot
                                               * A, the last resort, it looks like a slot written by a
                                               * tool that keeps no record */
    *sim_flash(off + 0x800u + below(size - 0x800u)) ^= (uint8_t)(1u << below(8u));
    story(" %lu:bad", (unsigned long)slot, 0);
  } else if (kind < 83u) {                    /* a boot loader, put there by mistake */
    uint8_t *f = make_image(2 + slot, size, 1);
    memcpy(sim_flash(off), f, size);
    cb->slot[slot].crc32 = sim_crc32(f, size);
    story(" %lu:fsbl", (unsigned long)slot, 0);
  } else if (kind < 92u) {                    /* an update cut short, as this loader leaves one */
    memset(sim_flash(off + 0x400u), 0xFF, 0x100u);
    memset(sim_flash(off + size / 2u), 0xFF, size - size / 2u);
    cb->slot[slot].state = N6_OTA_SLOT_EMPTY;
    story(" %lu:cut", (unsigned long)slot, 0);
  } else {                                    /* whole, but written by a tool that keeps no record */
    cb->slot[slot].crc32 ^= 0x5A5A5A5Au;
    story(" %lu:norecord", (unsigned long)slot, 0);
  }
}

static int g_staged_unfit;       /* what is staged is damaged, or is not an application */

static void stage_update(void)
{
  n6_ota_psram_hdr_t h;
  const uint32_t kind = below(100u);
  const uint32_t size = 0x1000u + (below(150000u) & ~3u);
  uint8_t *img = make_image(2, size, kind >= 85u);

  h.magic   = N6_OTA_PSRAM_HDR_MAGIC;
  h.size    = size;
  h.version = 7u;
  h.crc32   = sim_crc32(img, size);
  h.hdr_crc = sim_crc32(&h, 16u);
  memcpy(sim_psram(N6_OTA_PSRAM_BASE + N6_OTA_PSRAM_IMAGE_OFF), img, size);
  memcpy(sim_psram(N6_OTA_PSRAM_BASE), &h, sizeof(h));
  g_staged_unfit = (kind >= 70u);
  if ((kind >= 70u) && (kind < 85u)) {        /* damaged in the PSRAM */
    *sim_psram(N6_OTA_PSRAM_BASE + N6_OTA_PSRAM_IMAGE_OFF + below(size)) ^= 0x20u;
  } else if (kind < 70u) {
    know(img, size);
  }
  *sim_bkp(N6_OTA_TRIGGER_BKPR) = N6_OTA_TRIGGER_MAGIC;
  story(" stage(%lu,%lu)", (unsigned long)kind, (unsigned long)size);
}

static void stage_settings(void)
{
  static uint8_t data[N6_ASSET_MAX_BYTES];
  n6_asset_hdr_t h;
  const uint32_t len = 1u + below(1500u);

  for (uint32_t i = 0u; i < len; i++) { data[i] = (uint8_t)rnd(); }
  h.magic   = N6_ASSET_PSRAM_MAGIC;
  h.version = 1u + below(50u);
  h.len     = len;
  h.crc32   = sim_crc32(data, len);
  h.hdr_crc = sim_crc32(&h, 16u);
  memcpy(sim_psram(N6_ASSET_PSRAM_BASE + N6_ASSET_HDR_BYTES), data, len);
  memcpy(sim_psram(N6_ASSET_PSRAM_BASE), &h, sizeof(h));
  *sim_bkp(N6_ASSET_TRIGGER_BKPR) = N6_ASSET_TRIGGER_MAGIC;
  story(" settings(%lu)", (unsigned long)len, 0);
}

/* Is this copy of the stored settings whole: a header that checks out over a
 * payload that does? */
static int copy_whole(int c)
{
  n6_asset_hdr_t h;

  memcpy(&h, sim_flash(N6_ASSET_OFF(c)), sizeof(h));
  return (h.magic == N6_ASSET_MAGIC) && (h.hdr_crc == sim_crc32(&h, 16u)) && (h.len != 0u) &&
         (h.len <= N6_ASSET_MAX_BYTES) &&
         (sim_crc32(sim_flash(N6_ASSET_OFF(c) + N6_ASSET_HDR_BYTES), h.len) == h.crc32);
}

static int settings_whole(void) { return copy_whole(0) + copy_whole(1); }

/* Was anything in a slot erased by a flash operation from `first` on? */
static int slot_erased_since(uint32_t first)
{
  for (uint32_t i = first; (i < sim->ops) && (i < SIM_OPLOG); i++) {
    if ((sim->oplog[i].kind == 'E') && (n6_ota_slot_of_off(sim->oplog[i].addr) >= 0)) { return 1; }
  }
  return 0;
}

/* Was this copy's sector erased by a flash operation from `first` on? */
static int copy_erased_since(int c, uint32_t first)
{
  for (uint32_t i = first; (i < sim->ops) && (i < SIM_OPLOG); i++) {
    if ((sim->oplog[i].kind == 'E') && (sim->oplog[i].addr == N6_ASSET_OFF(c))) { return 1; }
  }
  return 0;
}

static void fuzz_boot(void)
{
  n6_ota_cb_t cb;
  const int has_cb = chance(70);
  int last_slot = -1, last_trial = 0, had_settings;
  int end = SIM_END_NONE;

  g_what = "boots";
  g_story[0] = '\0';
  forget_all();
  sim_init();
  sim->io1_high = (int)below(2u);
  memset(&cb, 0, sizeof(cb));
  cb.seq     = 1u + below(1000u);
  cb.active  = (uint8_t)below(2u);
  cb.pending = N6_OTA_NO_PENDING;
  fill_slot(0, &cb);
  if (has_cb || chance(30)) { fill_slot(1, &cb); }
  if (has_cb) {
    if (chance(25)) { cb.pending = (uint8_t)below(2u); cb.tries_left = (uint8_t)below(4u); }
    cb.magic = N6_OTA_CB_MAGIC;
    cb.crc   = n6_ota_cb_crc(&cb);
    memcpy(sim_flash(N6_OTA_CB_OFF(below(2u))), &cb, sizeof(cb));
    story(" cb(active %lu pending %lu)", cb.active, cb.pending);
  }
  if (chance(40)) {                           /* settings stored before, in one copy */
    stage_settings();
    (void)sim_boot();
    sim_ops_clear();
    g_story[0] = '\0';
  }
  had_settings = settings_whole();

  for (int step = 0; (step < 7) && (g_bad == 0); step++) {
    n6_ota_cb_t now;
    int slot, trial, only_whole;
    const uint32_t ops_before = sim->ops;

    /* what happens between this boot and the last */
    if ((step > 0) && chance(20)) { sim_power_cycle(); story(" power", 0, 0); }
    g_staged_unfit = 0;
    if (chance(25)) { stage_update(); }
    if (chance(30)) { stage_settings(); }
    if (chance(20)) { *sim_bkp(N6_OTA_CONFIRM_BKPR) = N6_OTA_CONFIRM_MAGIC; story(" confirm", 0, 0); }
    if (chance(10)) { *sim_bkp(2) = FORCEDFU; story(" askdfu", 0, 0); }
    for (unsigned r = 0; chance(30) && (r < 4u); r++) {     /* an application's own registers */
      static const uint8_t app[] = { 0, 3, 4, 5, 6, 7, 8, 9, 10, 23, 24, 25, 26, 27, 28, 29, 30, 31 };
      *sim_bkp(app[below(sizeof(app))]) = rnd();
    }
    sim->fail_map_call[0] = 0; sim->fail_map_run[0] = 0; sim->cut_at_op = 0u;
    if (chance(12)) {
      sim->fail_map_call[0] = 1 + (int)below(8u);
      sim->fail_map_run[0]  = (int)below(4u);
      story(" mapfail(%lu+%lu)", (unsigned long)sim->fail_map_call[0], (unsigned long)sim->fail_map_run[0]);
    }
    if (chance(15)) {                        /* as often in the first few operations as anywhere later */
      sim->cut_at_op = sim->ops + 1u + (chance(50) ? below(8u) : below(700u));
      story(" cut@%lu", sim->cut_at_op - sim->ops, 0);
    }
    /* the one copy of the settings that is whole, if there is exactly one */
    only_whole = (copy_whole(0) != copy_whole(1)) ? (copy_whole(0) ? 0 : 1) : -1;
    sim->dfu_idle_limit_ms = 70000u;                         /* longer than a trial is left in the bootloader */
    sim->app_feeds_wdg = 1;
    (void)sim_elapse(1500u + below(3000u));

    end = sim_boot();
    story(" ->%lu@%lX", (unsigned long)end, (end == SIM_END_APP) ? sim->boot_offset : 0ul);

    /* an update that is damaged, or is not an application, costs no slot */
    if (g_staged_unfit && slot_erased_since(ops_before)) {
      fail("a staged image that could not be an update had a slot erased for it", 0, 0);
    }
    /* an update on trial is never left in the bootloader with nobody there */
    if ((end == SIM_END_DFU_IDLE) && (sim->fail_map_call[0] == 0) && (cb_read(&now) >= 0) && (now.pending <= 1u)) {
      fail("in the bootloader for good, with slot %lu on trial", now.pending, 0);
    }
    /* a settings write never erases the only whole copy to make room */
    if ((only_whole >= 0) && copy_erased_since(only_whole, ops_before)) {
      fail("settings copy %lu was the only whole one, and was erased", (unsigned long)only_whole, 0);
    }

    if ((end == SIM_END_CRASH) || (end == SIM_END_HANG) || (end == SIM_END_ERROR) || (end == SIM_END_NONE)) {
      fail("the loader ended as %lu", (unsigned long)end, 0);
    }
    if (*sim_bkp(N6_OTA_APPLY_BKPR) == N6_OTA_APPLY_OK) { g_seen.applied++; *sim_bkp(N6_OTA_APPLY_BKPR) = 0u; }
    if (end == SIM_END_DFU_IDLE) { g_seen.bootloader++; }
    if (end == SIM_END_POWER_CUT) {
      g_seen.cuts++;
      sim->cut_at_op = 0u;
      sim_power_cycle();
      last_slot = -1;
      /* a write cut short never costs the settings that were whole */
      if ((had_settings > 0) && (settings_whole() == 0)) { fail("a cut left no whole copy of the settings", 0, 0); }
      had_settings = settings_whole();
      continue;
    }
    had_settings = settings_whole();
    if (sim->prog_dirty != 0u) { fail("%lu bytes programmed that were not erased", sim->prog_dirty, 0); }
    if (end != SIM_END_APP) {
      if ((end == SIM_END_RESET) || (end == SIM_END_WATCHDOG)) { last_slot = -1; }
      continue;
    }

    slot  = (sim->boot_offset == SLOT_B) ? 1 : 0;
    trial = (cb_read(&now) >= 0) && (now.pending == (uint8_t)slot);
    g_seen.entered++;
    g_seen.trials += (unsigned long)trial;

    if ((sim->boot_offset != SLOT_A) && (sim->boot_offset != SLOT_B)) { fail("jumped to %lX", sim->boot_offset, 0); }
    if (!slot_is_whole(sim->boot_offset)) { fail("jumped into slot %lu, which holds no whole application", (unsigned long)slot, 0); }
    if (trial != (sim->wdg_running != 0)) { fail("on trial %lu, watchdog running %lu", (unsigned long)trial, (unsigned long)sim->wdg_running); }
    /* the update, if one was applied, did not land on the image that ran last */
    if ((last_slot >= 0) && !last_trial && (sim->ops > ops_before) &&
        (sim_erased_in(last_slot ? SLOT_B : SLOT_A, N6_OTA_SLOT_SIZE) != 0u)) {
      fail("slot %lu was erased, and the board had last run from it", (unsigned long)last_slot, 0);
    }
    sim_ops_clear();
    last_slot  = slot;
    last_trial = trial;
  }

  /* Left alone from here, with resets and nothing else, it settles. */
  if (g_bad == 0) {
    uint32_t where = 0xFFFFFFFFu;
    int settled = 0;

    sim->fail_map_call[0] = 0; sim->cut_at_op = 0u;
    for (int i = 0; (i < 12) && !settled; i++) {
      sim->app_feeds_wdg = 1;
      (void)sim_elapse(2000u);
      sim->dfu_idle_limit_ms = 70000u;
      end = sim_boot();
      if (end == SIM_END_DFU_IDLE) { settled = 1; }
      if ((end == SIM_END_APP) && (sim->boot_offset == where) && !sim->wdg_running) { settled = 1; }
      if (end == SIM_END_APP) {
        where = sim->boot_offset;
        if (!slot_is_whole(where)) { fail("jumped into %lX, which holds no whole application", where, 0); }
      }
      if ((end == SIM_END_CRASH) || (end == SIM_END_HANG) || (end == SIM_END_ERROR)) { fail("the loader ended as %lu", (unsigned long)end, 0); }
    }
    if (!settled && (g_bad == 0)) { fail("twelve resets later it has not settled (last end %lu)", (unsigned long)end, 0); }
  }
}

/* Main. */

int main(int argc, char **argv)
{
  const uint32_t runs = (argc > 1) ? (uint32_t)strtoul(argv[1], NULL, 0) : 400u;
  const uint32_t seed = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 0) : 20261001u;
  const uint32_t only = (argc > 3) ? (uint32_t)strtoul(argv[3], NULL, 0) : 0u;
  const char    *kind = (argc > 4) ? argv[4] : "uploads and boots by turns";
  const int      ups  = (strcmp(kind, "uploads") == 0), boots = (strcmp(kind, "boots") == 0);

  g_seed = seed;
  for (g_run = 1u; (g_run <= runs) && (g_bad == 0); g_run++) {
    if ((only != 0u) && (g_run != only)) { continue; }
    g_x = (seed * 2654435761u) ^ (g_run * 40503u) ^ 0x9E3779B9u;
    if (g_x == 0u) { g_x = 1u; }
    if (ups || (!boots && ((g_run & 1u) != 0u))) { fuzz_upload(); } else { fuzz_boot(); }
  }
  if (g_bad == 0) {
    if (only != 0u) {
      printf("run %lu from seed %lu held\n", (unsigned long)only, (unsigned long)seed);
    } else {
      printf("%lu random runs from seed %lu, %s: every one held\n",
             (unsigned long)runs, (unsigned long)seed, kind);
    }
    printf("      uploads: %lu images accepted whole, %lu of them boot loaders, %lu requests refused\n",
           g_seen.images, g_seen.loaders, g_seen.refusals);
    printf("      boots: %lu applications entered, %lu on trial, %lu updates applied, %lu power cuts, "
           "%lu ends in the bootloader\n",
           g_seen.entered, g_seen.trials, g_seen.applied, g_seen.cuts, g_seen.bootloader);
  }
  return (g_bad != 0) ? 1 : 0;
}
