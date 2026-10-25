/**
 ******************************************************************************
 * @file    usb_descriptors.c
 * @brief   TinyUSB descriptors with a UVC video function in place of the
 *          vendor interface: device, configuration (video function and
 *          console) and strings. The serial number is the chip's unique ID
 *          with HAS_UNIQUE_ISERIAL_DESCRIPTOR, N6VND001 without.
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
/* bcdUSB 2.00: no vendor interface, so no MS OS 2.0 capability and no BOS. */
#define USB_BCD   0x0200

// Strings
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_VIDEO_IF,
  STRID_CDC_IF,
};

// Product string: -DBOARD_NAME if set. It arrives as unquoted tokens, so it
// is stringified here (two levels, to keep embedded spaces).
#ifdef BOARD_NAME
  #define USB_STRINGIFY2(x)   #x
  #define USB_STRINGIFY(x)    USB_STRINGIFY2(x)
  #define USB_PRODUCT_STRING  USB_STRINGIFY(BOARD_NAME)
#else
  #define USB_PRODUCT_STRING  "NeuroN6 Camera"
#endif

static char const* string_desc_arr[] = {
  (const char[]) {0x09, 0x04},   // 0: English (0x0409)
  "OhmLab",                      // 1
  USB_PRODUCT_STRING,            // 2 (override via -DBOARD_NAME=...)
  NULL,                          // 3 (serial generated below)
  "NeuroN6 Camera",              // 4
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

// No BOS: the video and CDC classes bind through their own class descriptors.
uint8_t const* tud_descriptor_bos_cb(void)
{
  return NULL;
}

// Configuration descriptor. The camera leads, as one interface association,
// and the console follows.
enum {
  ITF_NUM_VIDEO_CONTROL = 0,
  ITF_NUM_VIDEO_STREAMING,

  ITF_NUM_CDC_0,       // CDC Comm
  ITF_NUM_CDC_0_DATA,  // CDC Data

  ITF_NUM_TOTAL
};

// Video endpoint
#define EPNUM_VIDEO_IN    0x81

// CDC endpoints (must not collide with the video endpoint)
#define EPNUM_CDC_0_NOTIF 0x82
#define EPNUM_CDC_0_OUT   0x03
#define EPNUM_CDC_0_IN    0x83

// HS bulk max packet is 512, FS is 64
#if TUD_OPT_HIGH_SPEED
  #define CDC_BULK_MPS_HS     512
#endif
#define CDC_BULK_MPS_FS       64

// UVC: one MJPEG format, one frame size and one rate, the picture the
// example's camera and encoder produce.
#define UVC_FRAME_WIDTH      640
#define UVC_FRAME_HEIGHT     480
#define UVC_FRAME_RATE       60

// Frame intervals are in 100 ns units. A host reserves the uncompressed size
// for one frame, which no JPEG of that picture can exceed.
#define UVC_FRAME_INTERVAL   (10000000 / (UVC_FRAME_RATE))
#define UVC_MAX_FRAME_BYTES  ((UVC_FRAME_WIDTH) * (UVC_FRAME_HEIGHT) * 2)
#define UVC_MIN_BITRATE      ((UVC_FRAME_WIDTH) * (UVC_FRAME_HEIGHT) * 16)
#define UVC_MAX_BITRATE      (UVC_MIN_BITRATE * (UVC_FRAME_RATE))
// One isochronous packet per microframe; the class clamps its payload to
// CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE, which must be no larger. 1023 bytes is
// the largest packet valid at full speed as well as high: 8 MB/s at high speed.
#define UVC_EP_SIZE          1023
#define UVC_CLOCK_FREQUENCY       27000000
#define UVC_ENTITY_INPUT_TERMINAL   0x01
#define UVC_ENTITY_OUTPUT_TERMINAL  0x02

// The continuous frame descriptor with its smallest and largest interval
// equal: one rate, and the one the class picks when a host names none.
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

// A camera terminal into an output terminal, and one streaming interface
// whose alternate 1 carries the isochronous endpoint.
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

static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 500),

  // Video interfaces (FS)
  TUD_VIDEO_UVC_DESCRIPTOR(STRID_VIDEO_IF, EPNUM_VIDEO_IN, UVC_EP_SIZE),

  // CDC interface (FS)
  // ITF number (Comm), string index, EP notif (addr,size), EP OUT, EP IN, EP size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_IF,
                     EPNUM_CDC_0_NOTIF, 8,
                     EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN,
                     CDC_BULK_MPS_FS),
};

TU_VERIFY_STATIC(sizeof(desc_fs_configuration) == CONFIG_TOTAL_LEN,
                 "full-speed configuration descriptor length mismatch");

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 500),

  // Video interfaces (HS)
  TUD_VIDEO_UVC_DESCRIPTOR(STRID_VIDEO_IF, EPNUM_VIDEO_IN, UVC_EP_SIZE),

  // CDC interface (HS)
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
