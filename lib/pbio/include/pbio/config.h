

#ifndef _PBIO_CONFIG_H_
#define _PBIO_CONFIG_H_

// This file is defined per-platform. It should override the config values below as needed.
#include "pbioconfig.h"

// set to (1) if there is a port labeled "A" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_A
#define PBIO_CONFIG_HAS_PORT_A (0)
#endif

// set to (1) if there is a port labeled "B" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_B
#define PBIO_CONFIG_HAS_PORT_B (0)
#endif

// set to (1) if there is a port labeled "C" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_C
#define PBIO_CONFIG_HAS_PORT_C (0)
#endif

// set to (1) if there is a port labeled "D" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_D
#define PBIO_CONFIG_HAS_PORT_D (0)
#endif

// set to (1) if there is a port labeled "1" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_1
#define PBIO_CONFIG_HAS_PORT_1 (0)
#endif

// set to (1) if there is a port labeled "2" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_2
#define PBIO_CONFIG_HAS_PORT_2 (0)
#endif

// set to (1) if there is a port labeled "3" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_3
#define PBIO_CONFIG_HAS_PORT_3 (0)
#endif

// set to (1) if there is a port labeled "4" on the programmable brick
#ifndef PBIO_CONFIG_HAS_PORT_4
#define PBIO_CONFIG_HAS_PORT_4 (0)
#endif

// the number of built-in motor controllers in the programmable brick
#ifndef PBIO_CONFIG_NUM_MOTOR_CONTROLLER
#define PBIO_CONFIG_NUM_MOTOR_CONTROLLER (0)
#endif


// TODO: this doesn't really belong here. If the tacho is not built-in to the
// programmable brick, it could be a different value. We should probably pass
// this as a parameter to pbio_motor_set_constant_settings() instead.
#ifndef PBIO_MOTOR_COUNT_PER_ROT
/**
 * The number of motor tacho counts per one rotation. Use this value to convert
 * counts to degrees or rate to RPM, for example.
 */
#define PBIO_MOTOR_COUNT_PER_ROT 360
#endif

#endif // _PBIO_CONFIG_H_
