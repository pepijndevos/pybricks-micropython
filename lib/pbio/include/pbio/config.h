// SPDX-License-Identifier: MIT
// Copyright (c) 2019-2020 The Pybricks Authors

#ifndef _PBIO_CONFIG_H_
#define _PBIO_CONFIG_H_

// This file should be defined by applications that use the PBIO library.
#include "pbioconfig.h"

#ifndef PBIO_CONFIG_ENABLE_SYS
#define PBIO_CONFIG_ENABLE_SYS (0)
#endif

// Control loop time
#ifndef PBIO_CONFIG_CONTROL_LOOP_TIME_MS
#define PBIO_CONFIG_CONTROL_LOOP_TIME_MS (5)
#endif

// Angle differentiation time window, must be integer multiple of loop time
// and integer divisor of 1000.
#ifndef PBIO_CONFIG_DIFFERENTIATOR_WINDOW_MS
#define PBIO_CONFIG_DIFFERENTIATOR_WINDOW_MS (125)
#endif

#endif // _PBIO_CONFIG_H_
