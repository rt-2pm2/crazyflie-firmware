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
 *
 *
 *
 */
#ifndef __CONTROLLER_DD_OBJECTS_H__
#define __CONTROLLER_DD_OBJECTS_H__

#include "stabilizer_types.h"
#include "arm_math.h"

#include "estimator_dd_param.h"

#define DDCTRL_OUTPUTSIZE (4)

// DATA STRUCTURES

typedef struct DDController DDController;
struct DDController {
	// Measurement Data
	setpoint_t ctrl_setpoint;

	float Kxy[2];
	float Kz[2];
	float Katt[2];
	float Kyaw[2];

	float inputs[DDCTRL_OUTPUTSIZE];
	float phat[DDCTRL_OUTPUTSIZE];
	float phatminusalpha[DDCTRL_OUTPUTSIZE];
	float invbeta[DDCTRL_OUTPUTSIZE * DDCTRL_OUTPUTSIZE];

	arm_matrix_instance_f32 Phat;
	arm_matrix_instance_f32 PhatMinusAlpha; 
	arm_matrix_instance_f32 Inputs; 
	arm_matrix_instance_f32 InvBeta;

	bool initialized;
};
void DDController_Init(DDController* pc);
void DDController_SetSetpoint(DDController* pc, setpoint_t* ps);
void DDController_SetKxy(DDController* pc, const float k[2]);
void DDController_SetKz(DDController* pc, const float k[2]);
void DDController_SetKatt(DDController* pc, const float k[2]);
void DDController_SetKyaw(DDController* pc, const float k[2]);
void DDController_Step(DDController* pc,
		const state_t* ps, DDParams* pp, float deltaT);

/**
 * Control Action along a single axis
 */
float DDController_lin2angle(DDController* pc,
		const float setpoint[2],
		const float state_lin[2], const float state_ang[2],
		float alpha, float beta, float deltaT);

/**
 * Gets the computed motor control signals
 */
void DDController_getControls(DDController* pc,
		float m_ctrls[DDCTRL_OUTPUTSIZE]);
#endif //__CONTROLLER_DD_DATA_H__
