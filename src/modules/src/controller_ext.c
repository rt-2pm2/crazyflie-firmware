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


// ======================================================================
// 				controllerEXT	
void controllerEXTReset(void) {
}

void controllerEXTInit(void) {
	controllerEXTReset();
}

bool controllerEXTTest(void) {
	return true;
}


/**
 * In the update step of the DataDriven controller we just update the reference points..
 */
void controllerEXT(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors,
		const state_t *state,
		const uint32_t tick)
{
	// Extecute at the Attitude Rate
	if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		return;
	}

}
