/**
 ******************************************************************************
 * @file    NeuroN6_FrameQueue.h
 * @brief   Interface between a streaming application and the Wi-Fi library:
 *          frame queue, transport, network-host hand-over.
 *
 * The capture task posts each encoded frame and its TLV metadata to the queue;
 * the USB task sends it to the attached host: USB, or a network host handed
 * over through NeuroN6_NetAttach().
 *
 * The application defines everything here, as
 * Examples/Camera_Nia416Stream_WiFi_FreeRTOS does. The Wi-Fi library calls only
 * NeuroN6_NetAttach() and NeuroN6_IpAnnounceFrame(), so an application that
 * holds a host without streaming (Examples/WiFiApi_FreeRTOS) defines just those.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/MayaW476/LICENSE-MIT.txt.
 */

#ifndef NEURON6_FRAME_QUEUE_H
#define NEURON6_FRAME_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

/* Video transport, chosen at start-up; MayaW476_Start() takes either Wi-Fi
 * value. The stream goes to USB until a network host attaches. */
typedef enum {
  TRANSPORT_USB         = 0,   /* the USB vendor endpoint alone */
  TRANSPORT_WIFI_SOFTAP = 1,   /* the board hosts its own access point */
  TRANSPORT_WIFI_STA    = 2,   /* the board joins an existing network */
} n6_transport_t;

/* One encoded frame for the USB task. The pointers stay valid until
 * n6_video_tx_complete() releases the ping-pong buffer. jpeg_len 0 sends the
 * metadata alone and holds no buffer. */
typedef struct {
  const uint8_t *jpeg_ptr;
  uint32_t       jpeg_len;
  const uint8_t *meta_ptr;
  uint32_t       meta_len;
} n6_frame_msg_t;

/* Frame queue for USB and Wi-Fi alike, two deep like the ping-pong, created
 * before the tasks start. The capture task drops a frame it cannot post. */
extern QueueHandle_t g_n6_video_usb_queue;

/* Transport in use: the start-up choice, or TRANSPORT_USB if the Wi-Fi
 * library could not start (and until set). */
extern volatile n6_transport_t g_n6_active_transport;

/* Release a frame's ping-pong buffer. The USB task calls it once per JPEG
 * frame it sends, abandons or drains on a reconnect; the capture task reuses
 * a buffer only once it is released. */
void n6_video_tx_complete(void);

/* A network host, handed over by the Wi-Fi library after it accepts a
 * connection. The USB task drives it like USB: frames out through send(),
 * host commands (framed as in n6_host_contract.h) in through recv(). One host
 * at a time; while attached it gets the stream instead of USB.
 * Only the USB task calls these, and none may block. send() and recv() return
 * the bytes moved (0 when none can move now), < 0 once the connection is gone.
 * closed(): the USB task has let go and will not call the other two again, so
 * the library may close the connection. */
typedef struct {
  int32_t (*send)(const uint8_t *data, uint32_t len);
  int32_t (*recv)(uint8_t *data, uint32_t len);
  void    (*closed)(void);
} n6_net_link_t;

/* Hand a connected client to the USB task, which takes it within a
 * millisecond. False, leaving the connection with the caller, when a client
 * is already attached or the USB task is not running. */
bool NeuroN6_NetAttach(const n6_net_link_t *link);

/* The META_BOARD_IPV4 announce as one complete frame, for answering a
 * discovery probe (n6_host_contract.h). Writes it to `out` and returns its
 * length; 0 when `cap` is too small or the board has no address yet. */
uint32_t NeuroN6_IpAnnounceFrame(uint8_t *out, uint32_t cap);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_FRAME_QUEUE_H */
