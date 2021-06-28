/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 * Xu Research Group - Victor Freire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_lqr.h - LQR Controller Interface
 */
#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

// Frequency of control stages
#ifndef Z_PID_RATE
#define Z_PID_RATE RATE_500_HZ
#endif
#ifndef D9LQR_RATE
#define D9LQR_RATE RATE_25_HZ
#endif
#ifndef D6LQR_RATE
#define D6LQR_RATE RATE_25_HZ
#endif

// CF_CBF Parameters
#ifdef CF_CBF
#ifndef CF_CBF_K1
#define CF_CBF_K1       800.0f
#endif
#ifndef CF_CBF_K2
#define CF_CBF_K2       60.0f
#endif
#ifndef CF_CBF_X_MAX
#define CF_CBF_X_MAX    2.0f
#endif
#ifndef CF_CBF_X_MIN
#define CF_CBF_X_MIN    -2.0f
#endif
#ifndef CF_CBF_Y_MAX
#define CF_CBF_Y_MAX    2.0f
#endif
#ifndef CF_CBF_Y_MIN
#define CF_CBF_Y_MIN    -2.0f
#endif
#endif // CF_CBF

// Mode for the LQR
typedef enum {
  D9LQR = 0,  // 9-Dim model u = [T p q r]
  D6LQR = 1,  // 6-Dim model u = [T phi theta psi]
} lqr_mode_t;


void controllerLqrInit(void);
bool controllerLqrTest(void);
void controllerLqr(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

// Updates the [i][j] entry of the K matrix with value
void update_K_entry(const uint8_t i, const uint8_t j, float value);

#endif //__CONTROLLER_LQR_H__
