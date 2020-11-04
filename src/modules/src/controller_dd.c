/*
   The MIT License (MIT)

   Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   */

/*
   This controller is based on the following publication:
	"A novel approach to do something novel"

*/

#include <math.h>

#include "math3d.h"
#include "controller_dd_objects.h"
#include "controller_dd.h"

#include "num.h"
#include "param.h"
#include "log.h"

#define limitThrust(VAL) limitUint16(VAL)

// STATIC VARIABLES
DDController ddcontroller_;

// LOG AND PARAMETERS
static uint16_t motorPower[4];
static float motorSignals[4];

// Controller Gains
static float Kxy_[2] = {-4.0, -4.0};
static float Kz_[2] = {-25.0, -10.0};
static float Katt_[2] = {-16.0, -8.0};
static float Kyaw_[2] = {64, 16};


void controllerDD_update_gains() {
	// Setting Controller Gains
	DDController_SetKxy(&ddcontroller_, Kxy_);
	DDController_SetKz(&ddcontroller_, Kz_);
	DDController_SetKatt(&ddcontroller_, Katt_);
	DDController_SetKyaw(&ddcontroller_, Kyaw_);
}


// ======================================================================
// 				controllerDD	
void controllerDDReset(void) {
	DDController_Init(&ddcontroller_);
}

void controllerDDInit(void) {
	controllerDDReset();
	controllerDD_update_gains();
}

bool controllerDDTest(void) {
	return true;
}



/**
 * In the update step of the DataDriven controller we just update the reference points..
 */
void controllerDD(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors,
		const state_t *state,
		const uint32_t tick)
{
	// Extecute at the Attitude Rate
	if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		return;
	}

	DDController_SetSetpoint(&ddcontroller_, setpoint);
	
}


void controllerDD_Step(const state_t *ps, DDParams* pp, float T) {
	controllerDD_update_gains();
	DDController_Step(&ddcontroller_, ps, pp, T);

	// Get the input and actuate motors
	DDController_getControls(&ddcontroller_, motorSignals);
	
	for (int i = 0; i < 4; i++) {
		motorPower[i] = limitThrust(motorSignals[i] * 60000);
	}
}

void controllerDD_GetMotorPower(uint16_t v[4]) {
	for (int i = 0; i < 4; i++) {
		v[i] = motorPower[i];
	} 
}

void controllerDD_GetMotorSignals(float v[4]) {
	for (int i = 0; i < 4; i++) {
		v[i] = motorSignals[i];
	} 
}

PARAM_GROUP_START(ctrlDD_par)
	PARAM_ADD(PARAM_FLOAT, Kxy, &Kxy_[0])
	PARAM_ADD(PARAM_FLOAT, Kxy_d, &Kxy_[1])
	PARAM_ADD(PARAM_FLOAT, Kz, &Kz_[0])
	PARAM_ADD(PARAM_FLOAT, Kz_d, &Kz_[1])
	PARAM_ADD(PARAM_FLOAT, Katt, &Katt_[0])
	PARAM_ADD(PARAM_FLOAT, Katt_d, &Katt_[1])
	PARAM_ADD(PARAM_FLOAT, Kyaw, &Kyaw_[0])
	PARAM_ADD(PARAM_FLOAT, Kyaw_d, &Kyaw_[1])
PARAM_GROUP_STOP(ctrlDD_par)

LOG_GROUP_START(ctrlDD_log)
	LOG_ADD(LOG_UINT16, mpower0, &motorPower[0])
	LOG_ADD(LOG_UINT16, mpower1, &motorPower[1])
	LOG_ADD(LOG_UINT16, mpower2, &motorPower[2])
	LOG_ADD(LOG_UINT16, mpower3, &motorPower[3])
LOG_GROUP_STOP(ctrlDD_log)