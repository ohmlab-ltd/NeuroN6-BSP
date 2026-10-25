/**
  * @file    neuron6_camera_vd65g4.h
  * @brief   VD65G4 global-shutter image sensor as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_VD65G4_H
#define NEURON6_CAMERA_VD65G4_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "neuron6_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 0.56 MP colour global shutter, 804 x 704 RAW8 on one CSI-2 data lane; the driver
 * (Components/vd55g1) is ST's, unmodified.
 * Address, clock, lane rate and polarity default to the VD66GY module's, but with one
 * data lane a carrier that crosses the VD66GY's pair must not cross this one.
 * The part holds XSHUTDOWN low through ~20 kOhm until it boots: the carrier must pull
 * it up through < ~9 kOhm, or the sensor stays in shutdown and answers on no bus. */

/** @brief Sensor bus address, 8-bit form. */
#ifndef BSP_CAMERA_VD65G4_I2C_ADDRESS
#define BSP_CAMERA_VD65G4_I2C_ADDRESS   0x20U
#endif

/** @brief Model register value of a VD65G4 or VD55G4. */
#define BSP_CAMERA_VD65G4_ID            0x53354733U

/** @brief External clock frequency into the sensor, in Hz. */
#ifndef BSP_CAMERA_VD65G4_EXT_CLOCK_HZ
#define BSP_CAMERA_VD65G4_EXT_CLOCK_HZ  12000000U
#endif

/** @brief Data lane rate, in Mbps. */
#ifndef BSP_CAMERA_VD65G4_LANE_RATE_MBPS
#define BSP_CAMERA_VD65G4_LANE_RATE_MBPS 804U
#endif

/** @brief 1 if the module inverts the clock lane's polarity. */
#ifndef BSP_CAMERA_VD65G4_CLOCK_LANE_SWAP
#define BSP_CAMERA_VD65G4_CLOCK_LANE_SWAP 0U
#endif

/** @brief 1 if the module inverts the data lane's polarity. */
#ifndef BSP_CAMERA_VD65G4_DATA_LANE_SWAP
#define BSP_CAMERA_VD65G4_DATA_LANE_SWAP  0U
#endif

/* Resolution argument of BSP_CAMERA_Init. */
#define BSP_CAMERA_VD65G4_R320x240      0U /*!< QVGA.            */
#define BSP_CAMERA_VD65G4_R640x480      1U /*!< VGA.             */
#define BSP_CAMERA_VD65G4_R800x600      2U /*!< SVGA.            */
#define BSP_CAMERA_VD65G4_R804x704      3U /*!< The whole array. */

/** @brief Register the sensor; call before BSP_CAMERA_Init. RAW8 and the
  * BSP_CAMERA_VD65G4_R* resolutions only. */
int32_t BSP_CAMERA_VD65G4_Register(void);

/** @brief Colour-filter order (BSP_CAMERA_BAYER_*) read from the part; valid after
  * BSP_CAMERA_Init. A monochrome part reports RGGB. */
int32_t BSP_CAMERA_VD65G4_GetBayerOrder(uint32_t *pBayerOrder);

/** @brief Auto-exposure target brightness, 0 to 100. */
int32_t BSP_CAMERA_VD65G4_SetBrightness(int32_t Level);

/** @brief Driver context (VD55G1_Ctx_t, Components/vd55g1/vd55g1.h) for unwrapped calls. */
void *BSP_CAMERA_VD65G4_GetContext(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_VD65G4_H */
