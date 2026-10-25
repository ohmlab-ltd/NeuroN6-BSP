/* The boot loader's USB descriptors, FSBL/Core/Src/usb_descriptors.c, built
 * for a PC against the loader's own TinyUSB headers: what it answers a host
 * that asks for each of them. No USB is involved; run.sh builds and runs it.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>

#include "tusb.h"

static int fails, checks, opens;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)
/* A check of a v2.6.2 open finding: it fails until the finding is fixed, and
 * a pass fails the run so the mark is removed. */
#define OPEN(c) do { checks++; if (c) { fails++; printf("FAIL line %d: %s passes, but is listed as an open finding\n", __LINE__, #c); } \
                     else { opens++; printf("OPEN line %d: %s\n", __LINE__, #c); } } while (0)

/* What the loader's descriptor code calls. */
static tusb_speed_t g_speed;
tusb_speed_t tud_speed_get(void) { return g_speed; }

void n6_read_uid_words(uint32_t out[3])
{
  out[0] = 0x00130011u;
  out[1] = 0x42365010u;
  out[2] = 0x0037324Eu;
}

static unsigned total_len(const uint8_t *d) { return (unsigned)d[2] | ((unsigned)d[3] << 8); }

/* The largest bulk endpoint packet size in a configuration descriptor. */
static unsigned bulk_packet(const uint8_t *d)
{
  unsigned best = 0u;

  for (unsigned i = 0u; i < total_len(d); i += d[i]) {
    if ((d[i + 1] == TUSB_DESC_ENDPOINT) && ((d[i + 3] & 3u) == TUSB_XFER_BULK)) {
      const unsigned mps = (unsigned)d[i + 4] | ((unsigned)d[i + 5] << 8);
      if (mps > best) { best = mps; }
    }
    if (d[i] == 0u) { break; }
  }
  return best;
}

int main(void)
{
  static uint8_t cfg[2][256];
  const tusb_speed_t speeds[2] = { TUSB_SPEED_HIGH, TUSB_SPEED_FULL };
  const uint8_t *dev = tud_descriptor_device_cb();
  const uint8_t *qual = tud_descriptor_device_qualifier_cb();

  CHECK(dev[0] == 18u && dev[1] == TUSB_DESC_DEVICE);
  CHECK(dev[8] == 0x09u && dev[9] == 0x12u && dev[10] == 0x09u && dev[11] == 0x80u);   /* 1209:8009 */
  CHECK(qual[0] == 10u && qual[1] == TUSB_DESC_DEVICE_QUALIFIER);

  /* The configuration at each speed. */
  for (int s = 0; s < 2; s++) {
    const uint8_t *d;

    g_speed = speeds[s];
    d = tud_descriptor_configuration_cb(0);
    CHECK(d[0] == 9u && d[1] == TUSB_DESC_CONFIGURATION);
    CHECK(total_len(d) > 9u && total_len(d) <= sizeof(cfg[s]));
    memcpy(cfg[s], d, total_len(d));
    CHECK(d[4] == 3u);                                   /* the vendor interface and the serial port's two */
    CHECK(bulk_packet(d) == ((speeds[s] == TUSB_SPEED_HIGH) ? 512u : 64u));
  }
  CHECK(total_len(cfg[0]) == total_len(cfg[1]));

  /* The other speed's configuration: that speed's bytes, with the type a
   * host expects of an answer to that request (USB 2.0, 9.6.4). */
  for (int s = 0; s < 2; s++) {
    const uint8_t *other;

    g_speed = speeds[s];
    other = tud_descriptor_other_speed_configuration_cb(0);
    OPEN(other[0] == 9u && other[1] == TUSB_DESC_OTHER_SPEED_CONFIG);   /* v2.6.2 gives it the configuration's type */
    CHECK(total_len(other) == total_len(cfg[1 - s]));
    CHECK(memcmp(other + 2, cfg[1 - s] + 2, total_len(other) - 2u) == 0);
    CHECK(bulk_packet(other) == ((speeds[s] == TUSB_SPEED_HIGH) ? 64u : 512u));
    /* and asking for it leaves the configuration itself as it was */
    CHECK(tud_descriptor_configuration_cb(0)[1] == TUSB_DESC_CONFIGURATION);
    CHECK(memcmp(tud_descriptor_configuration_cb(0), cfg[s], total_len(cfg[s])) == 0);
  }

  /* The serial number: the chip's 96-bit identifier, as the uploader keys a board. */
  {
    const uint16_t *str = tud_descriptor_string_cb(3, 0x0409);
    char got[32] = "";

    CHECK(str != NULL && (str[0] >> 8) == TUSB_DESC_STRING && (str[0] & 0xFFu) == 2u * 24u + 2u);
    for (int i = 0; (str != NULL) && (i < 24); i++) { got[i] = (char)str[1 + i]; }
    CHECK(strcmp(got, "00130011423650100037324E") == 0);
  }

  printf("%d checks, %d failed, %d open findings reproduced (the boot loader's USB descriptors)\n",
         checks, fails, opens);
  return fails != 0;
}
