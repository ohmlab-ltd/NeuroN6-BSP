/**
  * @file    neuron6_errno.h
  * @brief   Error codes returned by the Neuro N6 BSP.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_ERRNO_H
#define NEURON6_ERRNO_H

/* Every fallible BSP function returns int32_t: BSP_ERROR_NONE on success or
 * one of the negative codes below. */

/* Common codes */
#define BSP_ERROR_NONE                     0
#define BSP_ERROR_NO_INIT                 -1
#define BSP_ERROR_WRONG_PARAM             -2
#define BSP_ERROR_BUSY                    -3
#define BSP_ERROR_PERIPH_FAILURE          -4
#define BSP_ERROR_COMPONENT_FAILURE       -5
#define BSP_ERROR_UNKNOWN_FAILURE         -6
#define BSP_ERROR_UNKNOWN_COMPONENT       -7
#define BSP_ERROR_BUS_FAILURE             -8
#define BSP_ERROR_CLOCK_FAILURE           -9
#define BSP_ERROR_MSP_FAILURE             -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED   -11

/* Bus codes */
#define BSP_ERROR_BUS_TRANSACTION_FAILURE    -100
#define BSP_ERROR_BUS_ARBITRATION_LOSS       -101
#define BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE    -102
#define BSP_ERROR_BUS_PROTOCOL_FAILURE       -103

#endif /* NEURON6_ERRNO_H */
