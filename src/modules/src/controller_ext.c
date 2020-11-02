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

#include "controller_ext.h"
#include "motors.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "semphr.h"

static uint32_t motorPower[4];
static SemaphoreHandle_t dataMutex;
static bool initialized;

// ======================================================================
// 				controllerEXT	
void controllerEXTReset(void) {
}

void controllerEXTInit(void) {
	controllerEXTReset();
	dataMutex = xSemaphoreCreateMutex();
	initialized = true;
}

bool controllerEXTTest(void) {
	return true;
}


/**
 * In the update step I set the motor power with the value received the last time from the crtp
 */
void controllerEXT(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors,
		const state_t *state,
		const uint32_t tick)
{
	if (!initialized) {
		controllerEXTInit();
	}
	// Extecute at the Attitude Rate
	if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		return;
	}

	if (dataMutex != NULL) {
		xSemaphoreTake(dataMutex, portMAX_DELAY);
	} else {
		DEBUG_PRINT("Mutex not initialized!\n");
		return;
	}
	motorsSetRatio(MOTOR_M1, motorPower[0]);
	motorsSetRatio(MOTOR_M2, motorPower[1]);
	motorsSetRatio(MOTOR_M3, motorPower[2]);
	motorsSetRatio(MOTOR_M4, motorPower[3]);
	xSemaphoreGive(dataMutex);
}

void controllerEXT_update_mtrs(const uint32_t m[4]) {
	if (!initialized) {
		controllerEXTInit();
	}

	if (dataMutex != NULL) {
		xSemaphoreTake(dataMutex, portMAX_DELAY);
		for (int i = 0; i < 4; i++) {
			motorPower[i] = m[i];
		}
		xSemaphoreGive(dataMutex);
	} else {
		DEBUG_PRINT("Mutex not initialized!\n");
		return;
	}
}
