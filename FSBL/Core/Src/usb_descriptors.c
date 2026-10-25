/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2026 Ohm Lab Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * SPDX-License-Identifier: MIT
 */

#include "tusb.h"
#include <string.h>

// ---------------- VID/PID ----------------
// pid.codes VID 0x1209: the bootloader is 0x1209:0x8009, the application
// 0x1209:0x8008. bcdDevice changes whenever the descriptors change, or Windows
// reuses a stale cached driver binding.
#define USB_VID   0x1209
#define USB_PID   0x8009
#define USB_BCD   0x0200

// ---------------- Strings ----------------
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_VENDOR_IF,
  STRID_CDC_IF,
};

static char const* string_desc_arr[] = {
  (const char[]) {0x09, 0x04},   // 0: English (0x0409)
  "OhmLab",                      // 1
  "Neuro N6 Bootloader",         // 2
  NULL,                          // 3 (serial generated below)
  "Vendor Stream",               // 4
  "CDC ACM",                     // 5
};

// ---------------- Device Descriptor ----------------
static tusb_desc_device_t const desc_device =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  // Composite device (IAD capable)
  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .idVendor           = USB_VID,
  .idProduct          = USB_PID,
  .bcdDevice          = 0x0200,   // v2 protocol (DFU_INFO/CRC32/NEXT/RESET)

  .iManufacturer      = STRID_MANUFACTURER,
  .iProduct           = STRID_PRODUCT,
  .iSerialNumber      = STRID_SERIAL,

  .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void)
{
  return (uint8_t const*) &desc_device;
}

// ---------------- Configuration Descriptor ----------------
// Interface numbers. The vendor interface is ITF 0.
enum {
  ITF_NUM_VENDOR = 0,

  ITF_NUM_CDC_0,       // CDC Comm
  ITF_NUM_CDC_0_DATA,  // CDC Data

  ITF_NUM_TOTAL
};

// ---------------- Vendor Endpoints ----------------
#define EPNUM_VENDOR_OUT  0x01
#define EPNUM_VENDOR_IN   0x81

// ---------------- CDC Endpoints ----------------
#define EPNUM_CDC_0_NOTIF 0x82
#define EPNUM_CDC_0_OUT   0x03
#define EPNUM_CDC_0_IN    0x83

// HS bulk max packet is 512, FS is 64
#if TUD_OPT_HIGH_SPEED
  #define VENDOR_BULK_MPS_HS  512
  #define CDC_BULK_MPS_HS     512
#endif
#define VENDOR_BULK_MPS_FS    64
#define CDC_BULK_MPS_FS       64

// Total length
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

  // -------- Vendor interface (FS) --------
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_FS),

  // -------- CDC interface (FS) --------
  // ITF number (Comm), string index, EP notif (addr,size), EP OUT, EP IN, EP size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_FS),
};

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

  // -------- Vendor interface (HS) --------
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_HS),

  // -------- CDC interface (HS) --------
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_HS),
};

static tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = USB_BCD,
  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*) &desc_device_qualifier;
}

uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index;
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration : desc_hs_configuration;
}
#endif

uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;

#if TUD_OPT_HIGH_SPEED
  if (tud_speed_get() == TUSB_SPEED_HIGH) return desc_hs_configuration;
#endif
  return desc_fs_configuration;
}

// ---------------- String Descriptor Callback ----------------
static uint16_t _desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  size_t chr_count;

  if (index == STRID_LANGID)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else if (index == STRID_SERIAL)
  {
    /* iSerial: the 96-bit chip UID as 24 hex digits, the same serial the
     * application reports, so host tools see one identity per board. Read
     * once. The fixed serial below is used if the UID reads blank. */
    extern void n6_read_uid_words(uint32_t out[3]);   /* main.c */
    static uint32_t uid[3];
    static bool uid_read = false;
    if (!uid_read) {
      n6_read_uid_words(uid);
      uid_read = true;
    }
    if ((uid[0] | uid[1] | uid[2]) != 0u &&
        (uid[0] & uid[1] & uid[2]) != 0xFFFFFFFFu) {
      static char const hex[] = "0123456789ABCDEF";
      uint8_t const len = 24;   /* 96 bits / 4 bits per hex char */
      for (int w = 0; w < 3; w++) {
        uint32_t v = uid[w];
        for (int j = 0; j < 8; j++) {
          _desc_str[1 + w * 8 + j] = (uint16_t)hex[(v >> (28 - j * 4)) & 0xFu];
        }
      }
      _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
      return _desc_str;
    }
    static char const serial[] = "N6VND001";
    uint8_t len = (uint8_t) strlen(serial);
    if (len > 32) len = 32;

    for (uint8_t i = 0; i < len; i++) _desc_str[1 + i] = serial[i];
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return _desc_str;
  }
    else
  {
    if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;

    const char* str = string_desc_arr[index];
    chr_count = strlen(str);
    if (chr_count > 32) chr_count = 32;

    for (size_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
