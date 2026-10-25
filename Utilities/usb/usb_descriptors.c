/**
 * @file    usb_descriptors.c
 * @brief   The board's TinyUSB descriptors, with BOS and MS OS 2.0 binding WinUSB.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Utilities/npu/LICENSE-MIT.txt.
 */

#include "tusb.h"
#include "stm32n6xx_hal.h"   /* UID_BASE for the unique iSerial */
#include <string.h>

// VID 0x1209 (pid.codes), PID 0x8008; the boot loader enumerates as 0x8009.
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
/* bcdUSB 0x0201 makes Windows fetch the BOS, whose MS OS 2.0 capability binds WinUSB. */
#if CFG_TUD_VIDEO
/* No vendor interface, so no BOS: the classes bind through their own descriptors. */
#define USB_BCD   0x0200
#else
#define USB_BCD   0x0201
#endif

// Strings
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_VENDOR_IF,
  STRID_CDC_IF,
  STRID_VIDEO_IF,
};

// Product string: -DBOARD_NAME (unquoted tokens, stringified here) or a fixed label.
#ifdef BOARD_NAME
  #define USB_STRINGIFY2(x)   #x
  #define USB_STRINGIFY(x)    USB_STRINGIFY2(x)
  #define USB_PRODUCT_STRING  USB_STRINGIFY(BOARD_NAME)
#elif CFG_TUD_VIDEO
  /* What a host shows in its camera picker. */
  #define USB_PRODUCT_STRING  "NeuroN6 Camera"
#else
  #define USB_PRODUCT_STRING  "NeuroN6 Vendor+CDC"
#endif

static char const* string_desc_arr[] = {
  (const char[]) {0x09, 0x04},   // 0: English (0x0409)
  "OhmLab",                      // 1
  USB_PRODUCT_STRING,            // 2 (override via -DBOARD_NAME=…)
  NULL,                          // 3 (serial generated below)
  "Vendor Stream",               // 4
  "CDC ACM",                     // 5
  "NeuroN6 Camera",              // 6 (UVC builds)
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
  /* Windows caches the MS OS 2.0 answer per VID/PID/bcdDevice: bump this whenever the BOS
   * or MS OS 2.0 descriptors change; never reuse a value (0x0100-0x0103 are taken). */
  .bcdDevice          = 0x0103,

  .iManufacturer      = STRID_MANUFACTURER,
  .iProduct           = STRID_PRODUCT,
  .iSerialNumber      = STRID_SERIAL,

  .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void)
{
  return (uint8_t const*) &desc_device;
}

#if !CFG_TUD_VIDEO
// BOS + MS OS 2.0: Windows 8.1+ binds WinUSB to the vendor interface (ITF 0) with no
// Zadig; the CDC keeps the inbox usbser driver. The BOS names a vendor request code,
// which Windows then sends (wIndex 7) for the descriptor set below.

#define VENDOR_REQUEST_MICROSOFT  0x01   /* bMS_VendorCode */

#define MS_OS_20_DESC_LEN  0xB2   /* 178 bytes, computed field-by-field below */

/* Set header, configuration subset, function subset (ITF 0), compatible ID "WINUSB" and
 * the DeviceInterfaceGUIDs property: host software can open the device by this GUID. */
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
#endif /* !CFG_TUD_VIDEO */

uint8_t const* tud_descriptor_bos_cb(void)
{
#if CFG_TUD_VIDEO
  /* UVC build: bcdUSB 0x0200, so a host never asks for a BOS. */
  return NULL;
#else
  return desc_bos;
#endif
}

#if !CFG_TUD_VIDEO

/* EP0 vendor requests: only the MS OS 2.0 descriptor fetch, anything else stalls. The
 * bulk command protocol (tud_vendor_rx_cb) is a separate path. */
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
#endif /* !CFG_TUD_VIDEO */

// Configuration descriptor
#if CFG_TUD_VIDEO
// UVC build: video first (one association, as UVC requires), then the console.
enum {
  ITF_NUM_VIDEO_CONTROL = 0,
  ITF_NUM_VIDEO_STREAMING,

  ITF_NUM_CDC_0,       // CDC Comm
  ITF_NUM_CDC_0_DATA,  // CDC Data

  ITF_NUM_TOTAL
};

#define EPNUM_VIDEO_IN    0x81
#else
// The vendor interface must stay ITF 0.
enum {
  ITF_NUM_VENDOR = 0,

  ITF_NUM_CDC_0,       // CDC Comm
  ITF_NUM_CDC_0_DATA,  // CDC Data

  ITF_NUM_TOTAL
};

// Vendor endpoints
#define EPNUM_VENDOR_OUT  0x01
#define EPNUM_VENDOR_IN   0x81
#endif

// CDC endpoints: must not collide with the vendor or video ones.
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

#if CFG_TUD_VIDEO
// UVC: one MJPEG format, frame size and rate, matching what the sensor and encoder
// produce; change these with the example's own resolution.
#ifndef UVC_FRAME_WIDTH
#define UVC_FRAME_WIDTH   640
#endif
#ifndef UVC_FRAME_HEIGHT
#define UVC_FRAME_HEIGHT  480
#endif
#ifndef UVC_FRAME_RATE
#define UVC_FRAME_RATE    60
#endif

// Video probe and commit count frame intervals in 100 ns units.
#define UVC_FRAME_INTERVAL   (10000000 / (UVC_FRAME_RATE))
// Per-frame reservation: the uncompressed size, which no JPEG of the picture exceeds.
#define UVC_MAX_FRAME_BYTES  ((UVC_FRAME_WIDTH) * (UVC_FRAME_HEIGHT) * 2)
#define UVC_MIN_BITRATE      ((UVC_FRAME_WIDTH) * (UVC_FRAME_HEIGHT) * 16)
#define UVC_MAX_BITRATE      (UVC_MIN_BITRATE * (UVC_FRAME_RATE))
// One isochronous packet per microframe (8 MB/s at high speed): 1023 is the largest
// valid at both speeds. CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE must not exceed it.
#define UVC_EP_SIZE          1023
// The usual UVC clock (27 MHz); the camera has two terminals and no units between.
#define UVC_CLOCK_FREQUENCY       27000000
#define UVC_ENTITY_INPUT_TERMINAL   0x01
#define UVC_ENTITY_OUTPUT_TERMINAL  0x02

/* A continuous frame descriptor with equal minimum and maximum intervals: one rate, the
 * one the video class picks when a host commits without naming one. */
#define UVC_FRM_DESC_LEN  TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_CONT_LEN
#define UVC_VS_DESC_LEN   (TUD_VIDEO_DESC_CS_VS_FMT_MJPEG_LEN \
                           + UVC_FRM_DESC_LEN \
                           + TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN)

#define TUD_VIDEO_UVC_DESC_LEN ( \
    TUD_VIDEO_DESC_IAD_LEN \
  + TUD_VIDEO_DESC_STD_VC_LEN \
  + (TUD_VIDEO_DESC_CS_VC_LEN + 1)   /* one streaming interface in the collection */ \
  + TUD_VIDEO_DESC_CAMERA_TERM_LEN \
  + TUD_VIDEO_DESC_OUTPUT_TERM_LEN \
  + TUD_VIDEO_DESC_STD_VS_LEN        /* alternate 0: no bandwidth */ \
  + (TUD_VIDEO_DESC_CS_VS_IN_LEN + 1) \
  + UVC_VS_DESC_LEN \
  + TUD_VIDEO_DESC_STD_VS_LEN        /* alternate 1: the endpoint */ \
  + 7)

// Camera terminal to output terminal; streaming alternate 1 carries the isochronous
// endpoint, alternate 0 (a host that stopped watching) has none.
#define TUD_VIDEO_UVC_DESCRIPTOR(_stridx, _epin, _epsize) \
  TUD_VIDEO_DESC_IAD(ITF_NUM_VIDEO_CONTROL, 2, _stridx), \
  TUD_VIDEO_DESC_STD_VC(ITF_NUM_VIDEO_CONTROL, 0, _stridx), \
    TUD_VIDEO_DESC_CS_VC(/* UVC 1.5 */ 0x0150, \
        TUD_VIDEO_DESC_CAMERA_TERM_LEN + TUD_VIDEO_DESC_OUTPUT_TERM_LEN, \
        UVC_CLOCK_FREQUENCY, ITF_NUM_VIDEO_STREAMING), \
      TUD_VIDEO_DESC_CAMERA_TERM(UVC_ENTITY_INPUT_TERMINAL, 0, 0, \
          /*focal min*/0, /*focal max*/0, /*focal*/0, /*bmControls*/0), \
      TUD_VIDEO_DESC_OUTPUT_TERM(UVC_ENTITY_OUTPUT_TERMINAL, VIDEO_TT_STREAMING, 0, \
          UVC_ENTITY_INPUT_TERMINAL, 0), \
  TUD_VIDEO_DESC_STD_VS(ITF_NUM_VIDEO_STREAMING, 0, 0, _stridx), \
    TUD_VIDEO_DESC_CS_VS_INPUT(/*bNumFormats*/1, UVC_VS_DESC_LEN, _epin, /*bmInfo*/0, \
        /*bTerminalLink*/UVC_ENTITY_OUTPUT_TERMINAL, /*bStillCaptureMethod*/0, \
        /*bTriggerSupport*/0, /*bTriggerUsage*/0, /*bmaControls(1)*/0), \
      TUD_VIDEO_DESC_CS_VS_FMT_MJPEG(/*bFormatIndex*/1, /*bNumFrameDescriptors*/1, \
          /*bmFlags: no fixed sample size*/0, /*bDefaultFrameIndex*/1, 0, 0, 0, /*bCopyProtect*/0), \
        TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_CONT(/*bFrameIndex*/1, /*bmCapabilities*/0, \
            UVC_FRAME_WIDTH, UVC_FRAME_HEIGHT, UVC_MIN_BITRATE, UVC_MAX_BITRATE, \
            UVC_MAX_FRAME_BYTES, /*default*/UVC_FRAME_INTERVAL, \
            /*min*/UVC_FRAME_INTERVAL, /*max*/UVC_FRAME_INTERVAL, /*step*/0), \
      TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING(VIDEO_COLOR_PRIMARIES_BT709, \
          VIDEO_COLOR_XFER_CH_BT709, VIDEO_COLOR_COEF_SMPTE170M), \
  TUD_VIDEO_DESC_STD_VS(ITF_NUM_VIDEO_STREAMING, 1, 1, _stridx), \
    TUD_VIDEO_DESC_EP_ISO(_epin, _epsize, 1)

TU_VERIFY_STATIC(CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE <= UVC_EP_SIZE,
                 "the streaming buffer cannot exceed the endpoint");

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VIDEO_UVC_DESC_LEN + TUD_CDC_DESC_LEN)
#else
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN)
#endif

static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

#if CFG_TUD_VIDEO
  // Video interfaces
  TUD_VIDEO_UVC_DESCRIPTOR(STRID_VIDEO_IF, EPNUM_VIDEO_IN, UVC_EP_SIZE),
#else
  // Vendor interface
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_FS),
#endif

  // CDC interface
  // ITF number (Comm), string index, EP notif (addr,size), EP OUT, EP IN, EP size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_FS),
};

/* A declared length that differs from the assembled one enumerates as nothing. */
TU_VERIFY_STATIC(sizeof(desc_fs_configuration) == CONFIG_TOTAL_LEN,
                 "full-speed configuration descriptor length mismatch");

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

#if CFG_TUD_VIDEO
  // Video interfaces
  TUD_VIDEO_UVC_DESCRIPTOR(STRID_VIDEO_IF, EPNUM_VIDEO_IN, UVC_EP_SIZE),
#else
  // Vendor interface
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_IF,
                        EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN,
                        VENDOR_BULK_MPS_HS),
#endif

  // CDC interface
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_HS),
};

TU_VERIFY_STATIC(sizeof(desc_hs_configuration) == CONFIG_TOTAL_LEN,
                 "high-speed configuration descriptor length mismatch");

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

/* The other speed's configuration, typed OTHER_SPEED_CONFIGURATION (USB 2.0, 9.6.4):
 * TinyUSB sends it as it is, so the type is set here. */
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
    /* The 96-bit UID as 24 hex characters, distinct per board. Windows may key driver
     * bindings to it; without HAS_UNIQUE_ISERIAL_DESCRIPTOR every board is N6VND001. */
    static const char hex[] = "0123456789ABCDEF";
    /* The UID is in BSEC, gated on APB4 and clocked nowhere else: unclocked, the read
     * bus-faults. Enable it (atomic set) and cache the UID once. */
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
    /* A fixed serial, so Windows bindings keyed to "N6VND001" keep working. */
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
