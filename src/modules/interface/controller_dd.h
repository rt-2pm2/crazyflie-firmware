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
 * controller_dd.h - DataDriven Controller Interface
 */
#ifndef __CONTROLLER_DD_H__
#define __CONTROLLER_DD_H__


#include "estimator_dd_param.h"
#include "stabilizer_types.h"

void controllerDDInit(void);
bool controllerDDTest(void);
void controllerDD(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

void controllerDD_Step(const state_t *state, DDParams* params,
		float T);

/**
 * Return the motor power assigned to each motor by the controller
 */
void controllerDD_GetMotorPower(uint16_t v[4]);

/**
 * Return the motor control signals assigned to each motor by the controller
 */
void controllerDD_GetMotorSignals(float v[4]);


#endif //__CONTROLLER_DD_H__
