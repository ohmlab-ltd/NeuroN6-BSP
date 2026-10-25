/**
 ******************************************************************************
 * @file    usb_descriptors.c
 * @brief   TinyUSB descriptors: device, configuration (vendor interface and
 *          console), strings, and the BOS and MS OS 2.0 descriptors that bind
 *          the vendor interface to WinUSB. The serial number is the chip's
 *          unique ID with HAS_UNIQUE_ISERIAL_DESCRIPTOR, N6VND001 without.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Utilities/npu/LICENSE-MIT.txt.
 */

#include "tusb.h"
#include "stm32n6xx_hal.h"   /* UID_BASE for the unique iSerial */
#include <string.h>

// VID 0x1209 (pid.codes) / PID 0x8008; the FSBL bootloader is 0x1209:0x8009.
// Override with -DBOARD_VENDORID=0x... -DBOARD_PRODUCTID=0x...
#ifdef BOARD_VENDORID
  #define USB_VID  BOARD_VENDORID
#else
  #define USB_VID  0x1209
#endif
#ifdef BOARD_PRODUCTID
  #define USB_PID  BOARD_PRODUCTID
#else
  #define USB_PID  0x8008
#endif
/* bcdUSB 0x0201 or higher makes Windows request the BOS (MS OS 2.0 below);
 * without it the vendor interface needs a manual WinUSB install. */
#define USB_BCD   0x0201

// Strings
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_VENDOR_IF,
  STRID_CDC_IF,
};

// Product string: -DBOARD_NAME if set. It arrives as unquoted tokens, so it
// is stringified here (two levels, to keep embedded spaces).
#ifdef BOARD_NAME
  #define USB_STRINGIFY2(x)   #x
  #define USB_STRINGIFY(x)    USB_STRINGIFY2(x)
  #define USB_PRODUCT_STRING  USB_STRINGIFY(BOARD_NAME)
#else
  #define USB_PRODUCT_STRING  "NeuroN6 Vendor+CDC"
#endif

static char const* string_desc_arr[] = {
  (const char[]) {0x09, 0x04},   // 0: English (0x0409)
  "OhmLab",                      // 1
  USB_PRODUCT_STRING,            // 2 (override via -DBOARD_NAME=...)
  NULL,                          // 3 (serial generated below)
  "Vendor Stream",               // 4
  "CDC ACM",                     // 5
};

// Device descriptor
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
  /* Windows caches the MS OS descriptor result per VID/PID/bcdDevice: bump
   * bcdDevice whenever the BOS or MS OS 2.0 descriptors change. */
  .bcdDevice          = 0x0102,

  .iManufacturer      = STRID_MANUFACTURER,
  .iProduct           = STRID_PRODUCT,
  .iSerialNumber      = STRID_SERIAL,

  .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void)
{
  return (uint8_t const*) &desc_device;
}

// BOS + MS OS 2.0: Windows 8.1 and later bind WinUSB to the vendor interface
// (ITF 0) by themselves; the CDC interfaces keep the inbox usbser driver.
// Windows reads the BOS, then fetches the descriptor set below with vendor
// request VENDOR_REQUEST_MICROSOFT, wIndex 7.

#define VENDOR_REQUEST_MICROSOFT  0x01   /* bMS_VendorCode, an EP0 request */

#define MS_OS_20_DESC_LEN  0xB2   /* 178 bytes, computed field-by-field below */

/* MS OS 2.0 set: header, configuration subset, function subset (vendor ITF 0),
 * compatible ID "WINUSB" and the DeviceInterfaceGUIDs property. Host software
 * opens the device by that GUID, not by VID/PID. */
static uint8_t const desc_ms_os_20[] = {
  // Set header: wLength=10, wDescriptorType=MSOS20_SET_HEADER_DESCRIPTOR,
  // dwWindowsVersion=0x06030000 (8.1+), wTotalLength=MS_OS_20_DESC_LEN
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(0x0000),
  U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

  // Configuration subset header: wLength=8, wDescriptorType=MSOS20_SUBSET_HEADER_CONFIGURATION,
  // bConfigurationValue=0 (i.e. configuration index), bReserved, wTotalLength
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(0x0001),
  0x00, 0x00, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

  // Function subset header: wLength=8, wDescriptorType=MSOS20_SUBSET_HEADER_FUNCTION,
  // bFirstInterface=ITF_NUM_VENDOR (0), bReserved, wSubsetLength
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(0x0002),
  0x00 /* ITF_NUM_VENDOR */, 0x00, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

  // Compatible ID descriptor: wLength=20, wDescriptorType=MSOS20_FEATURE_COMPATBLE_ID,
  // CompatibleID "WINUSB\0\0", SubCompatibleID zeros
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(0x0003),
  'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // Registry property descriptor: wLength = 2+2+2+2+42+2+80 = 132 (0x84),
  // wDescriptorType=MSOS20_FEATURE_REG_PROPERTY,
  // wPropertyDataType=7 (REG_MULTI_SZ), wPropertyNameLength=42
  U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(0x0004), U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
  // PropertyName: "DeviceInterfaceGUIDs" UTF-16LE + NUL (21 chars = 42 bytes)
  'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0, 'I',0, 'n',0, 't',0, 'e',0,
  'r',0, 'f',0, 'a',0, 'c',0, 'e',0, 'G',0, 'U',0, 'I',0, 'D',0, 's',0, 0,0,
  // wPropertyDataLength = 80 (38-char GUID string + NUL + MULTI_SZ NUL, UTF-16LE)
  U16_TO_U8S_LE(0x0050),
  // PropertyData: "{0A4F8CB3-F257-401A-8BFB-375224C17089}" + double NUL
  '{',0, '0',0, 'A',0, '4',0, 'F',0, '8',0, 'C',0, 'B',0, '3',0, '-',0,
  'F',0, '2',0, '5',0, '7',0, '-',0, '4',0, '0',0, '1',0, 'A',0, '-',0,
  '8',0, 'B',0, 'F',0, 'B',0, '-',0, '3',0, '7',0, '5',0, '2',0, '2',0,
  '4',0, 'C',0, '1',0, '7',0, '0',0, '8',0, '9',0, '}',0, 0,0, 0,0,
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 descriptor size mismatch");

#define BOS_TOTAL_LEN  (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static uint8_t const desc_bos[] = {
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const* tud_descriptor_bos_cb(void)
{
  return desc_bos;
}

/* EP0 vendor requests: only the MS OS 2.0 descriptor fetch is answered. */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  if (stage != CONTROL_STAGE_SETUP) { return true; }   /* DATA/ACK: nothing to do */

  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
      request->bRequest == VENDOR_REQUEST_MICROSOFT &&
      request->wIndex == 7) {
    uint16_t total_len;
    memcpy(&total_len, desc_ms_os_20 + 8, 2);
    return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20, total_len);
  }

  return false;   /* unknown vendor request -> stall */
}

// Configuration descriptor. The vendor interface must stay at 0: the MS OS
// 2.0 function subset above hardcodes bFirstInterface = 0.
enum {
  ITF_NUM_VENDOR = 0,

  ITF_NUM_CDC_0,       // CDC Comm
  ITF_NUM_CDC_0_DATA,  // CDC Data

  ITF_NUM_TOTAL
};

// Vendor endpoints
#define EPNUM_VENDOR_OUT  0x01
#define EPNUM_VENDOR_IN   0x81

// CDC endpoints (must not collide with the vendor pair)
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

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 500),

  // Vendor interface (FS)
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_FS),

  // CDC interface (FS)
  // ITF number (Comm), string index, EP notif (addr,size), EP OUT, EP IN, EP size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_FS),
};

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 500),

  // Vendor interface (HS)
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_HS),

  // CDC interface (HS)
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

/* Other-speed configuration (USB 2.0, 9.6.4): the other speed's configuration
 * descriptor, retyped here because TinyUSB sends it as returned. */
static uint8_t desc_other_speed_configuration[CONFIG_TOTAL_LEN];

uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index;
  memcpy(desc_other_speed_configuration,
         (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration : desc_hs_configuration,
         CONFIG_TOTAL_LEN);
  desc_other_speed_configuration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
  return desc_other_speed_configuration;
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

// String descriptors
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
#ifdef HAS_UNIQUE_ISERIAL_DESCRIPTOR
    /* The 96-bit UID as 24 hex characters, unique per board. Drop
     * -DHAS_UNIQUE_ISERIAL_DESCRIPTOR for a fixed serial instead. */
    static const char hex[] = "0123456789ABCDEF";
    /* The UID is in BSEC, clock-gated on APB4 and enabled nowhere else:
     * reading it unclocked faults. Clock it, then read the UID once. */
    static uint32_t uid[3];
    static bool uid_valid = false;
    if (!uid_valid) {
      __HAL_RCC_BSEC_CLK_ENABLE();
      volatile uint32_t const *uid_words = (volatile uint32_t const *)UID_BASE;
      for (int w = 0; w < 3; w++) uid[w] = uid_words[w];
      uid_valid = true;
    }
    uint8_t const len = 24;   /* 96 bits / 4 bits per hex char */
    for (int w = 0; w < 3; w++) {
      uint32_t v = uid[w];
      for (int j = 0; j < 8; j++) {
        _desc_str[1 + w * 8 + j] = (uint16_t)hex[(v >> (28 - j * 4)) & 0xFu];
      }
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return _desc_str;
#else
    /* Fixed serial, the same on every board, so a Windows driver binding
     * keyed to it survives firmware updates. */
    static char const serial[] = "N6VND001";
    uint8_t len = (uint8_t) strlen(serial);
    if (len > 32) len = 32;

    for (uint8_t i = 0; i < len; i++) _desc_str[1 + i] = serial[i];
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return _desc_str;
#endif
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
