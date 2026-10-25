/**
  ******************************************************************************
  * @file    NeuroN6.h
  * @brief   Camera-crop helpers for the post-processing in this folder.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  ******************************************************************************
  *
  * Maps a detection from the network input onto the camera frame through the
  * camera crop; the application defines the crop state.
  *
  * On a case-insensitive file system this name and the BSP's "neuron6.h" are
  * the same: vnd_meta_nia.c finds this one beside it, and this folder must
  * come after the BSP's on the include path.
  */

#ifndef NEURON6_POSTPROCESS_CORE_H
#define NEURON6_POSTPROCESS_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The camera window the network sees, in camera pixels. */
extern uint16_t g_nn_crop_x;
extern uint16_t g_nn_crop_y;
extern uint16_t g_nn_crop_w;
extern uint16_t g_nn_crop_h;

/* The whole camera frame, in pixels. */
extern uint16_t g_camera_full_w;
extern uint16_t g_camera_full_h;

/* Network-input normalised coordinate to camera-frame normalised. */
static inline float nn_norm_to_cam_norm_x(float xn)
{
  return ((float)g_nn_crop_x + xn * (float)g_nn_crop_w) / (float)g_camera_full_w;
}
static inline float nn_norm_to_cam_norm_y(float yn)
{
  return ((float)g_nn_crop_y + yn * (float)g_nn_crop_h) / (float)g_camera_full_h;
}

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_POSTPROCESS_CORE_H */
