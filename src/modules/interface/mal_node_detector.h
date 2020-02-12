/**
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
 *
 */

#ifndef __MAL_NODE_DETECTOR_H__
#define __MAL_NODE_DETECTOR_H__

#include "cf_math.h"
#include "stabilizer_types.h"

#define NUM_ANCHORS (8)

// Indexes to access the quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, KC_STATE_DIM
} StateIdx_t;


struct AnchorData {
	distanceMeasurement_t data;
	uint8_t id;
	uint32_t tk_timestamp;
};

struct Actuation {
	float T;
	float roll;
	float pitch;
}

// The data used by the algorithm.
typedef struct {
  /**
   * The internally-estimated state is:
   * - X, Y, Z: the quad's position in the global frame
   */
  float State[KC_STATE_DIM];


	/*
	 * Anchor Position
	 */
	float AnchorPos[NUM_ANCHORS][3];

	/*
	 * Reconstruct
	 */
	float Reconstruct[4][6];

} InternalData_t;


void kalmanCoreInit(kalmanCoreData_t* this);

/*  - Measurement updates based on sensors */

// Barometer
void kalmanCoreUpdateWithBaro(kalmanCoreData_t* this, float baroAsl, bool quadIsFlying);

// Absolute height measurement along the room Z
void kalmanCoreUpdateWithAbsoluteHeight(kalmanCoreData_t* this, heightMeasurement_t* height);

// Direct measurements of Crazyflie position
void kalmanCoreUpdateWithPosition(kalmanCoreData_t* this, positionMeasurement_t *xyz);

// Direct measurements of Crazyflie pose
void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose);

// Distance-to-point measurements
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d);

// Measurements of a UWB Tx/Rx
void kalmanCoreUpdateWithTDOA(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa);

// Measurements of flow (dnx, dny)
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);

// Measurements of TOF from laser sensor
void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Measurement of yaw error (outside measurement Vs current estimation)
void kalmanCoreUpdateWithYawError(kalmanCoreData_t *this, yawErrorMeasurement_t *error);

// Measurement of sweep angles from a Lighthouse base station
void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *angles, const uint32_t tick);

/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
void kalmanCorePredict(kalmanCoreData_t* this, float thrust, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying);

void kalmanCoreAddProcessNoise(kalmanCoreData_t* this, float dt);

/*  - Finalization to incorporate attitude error into body attitude */
void kalmanCoreFinalize(kalmanCoreData_t* this, uint32_t tick);

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc, uint32_t tick);

void kalmanCoreDecoupleXY(kalmanCoreData_t* this);

#endif // __KALMAN_CORE_H__
