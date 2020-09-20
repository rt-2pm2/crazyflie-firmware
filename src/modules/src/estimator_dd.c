/*
 *
 * Copyright (c) 2020 Luigi Pannocchi 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"

#include "usec_time.h"

#include "estimator_dd_param.h"
#include "estimator_dd_objects.h"
#include "estimator_dd.h"

#include "controller_dd.h"

#include "debug.h"

#define CTRL_THRESHOLD (0.1f)

// STATIC VARIABLES
static float gains_x[2] = {1,1};
static float gains_y[2] = {1,1};
static float gains_2d[DDESTPAR_GAINS2DSIZE];


// ==================================================================
//			GLOBALS
// Global counter of received sensor messages 
unsigned long int msg_counter = 0;

/**
 * Estimator Structure
 */
static DDEstimator ddestimator_;
static DDParamEstimator ddparamestimator_;

static float state_vec[DDEST_FULLSTATESIZE];

// ==================================================================
// Helpers
bool control_valid(const float v[4]) {
	bool out = false;

	float norm = 0.0;
	float temp;
	for (int i = 0; i < 4; i++) {
		temp = powf(v[i], 2); 
		norm += temp;
	}

	if (norm > CTRL_THRESHOLD) {
		out = true;
	} else {
		out = false;
	}

	return out;
}

void  estimatorDD_SetGains(float gains_x[2], float gains_y[2],
		float gains_2d[DDESTPAR_GAINS2DSIZE]) {
	DDParamEstimator_SetGains(&ddparamestimator_, gains_x,
			gains_y, gains_2d);
}

// ===================================================================
// 				DDEstimator
//
//
// This is the data structure containing the state of the vehicle that 
// should be filled by the estimator.
//
// state_t  {
//	    attitude_t attitude; (In odd CF2 frame)
//	    quaternion_t attitudeQuaternion;
//	    point_t position;         // m
//	    velocity_t velocity;      // m/s
//	    acc_t acc;                // Gs (acc.z - gravity)
// }
//
// attitude_t = {
// 	uint32_t timestamp
// 	float roll;
// 	float pitch;
// 	float yaw;
// }
//
// quaternion_t {
// 	uint32_t timestamp
// 	float x;
// 	float y;
// 	float z;
// 	float w;
// }
//
// point_t = velocity_t = acc_t {
// 	uint32_t timestamp
// 	float x;
// 	float y;
// 	float z;
// }


// ==================================================================
// 				PRIVATE
float preproc_angle(float angle) {
	float processed = 0;

	processed = -angle/((angle - M_PI_F) * (angle + M_PI_F));

	return processed;
}

void attitude_preprocessing(float rpy_dest[3], const quaternion_t q) {

	struct quat q_src = {q.x, q.y, q.z, q.w}; 
	struct vec rpy = quat2rpy(q_src);

	rpy_dest[0] = rpy.x;
	rpy_dest[1] = rpy.y;
	rpy_dest[2] = rpy.z;
}

// ==================================================================



void estimatorDDInit(void) {
	// Initialize the estimators
	DDEstimator_Init(&ddestimator_);
	DDParamEstimator_Init(&ddparamestimator_);
	
	//XXX I initialize the gains2d here
};

bool estimatorDDTest(void) {
	return true;
}

void estimatorDD(state_t *state,
		sensorData_t *sensors,
		control_t *control,
		const uint32_t tick) {
	if (!RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
		return;
	}

	if (msg_counter % 300 == 0) {
		DDEstimator_GetState(&ddestimator_, state_vec);
		DEBUG_PRINT("ATT %f %f %f\n",
				(double)(state_vec[DDEST_ROLL] * 180.0f / M_PI_F),
				(double)(state_vec[DDEST_PITCH] * 180.0f / M_PI_F),
				(double)(state_vec[DDEST_YAW] * 180.0f / M_PI_F));
		DEBUG_PRINT("POS %3.1f %3.1f %3.1f\n",
				(double)state_vec[DDEST_X],
				(double)state_vec[DDEST_Y],
				(double)state_vec[DDEST_Z]);

	}
}


bool estimatorDD_Step(state_t *state,
		const float controls[4],
		const uint32_t tick) {
	bool updated = false;

	// Here we can control the rate
	if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
		// Update the gains
		estimatorDD_SetGains(gains_x, gains_y, gains_2d);

		// XXX Check if the state has been updated
		updated = DDEstimator_Step(&ddestimator_);

		if (updated) {
			// Export the estimated state into the system.
			DDEstimator_ExportState(&ddestimator_, state);

			float deltaT = 0;
			DDEstimator_GetMeasuresTimeInterval(
					&ddestimator_, &deltaT);

			// Check if the controller is issuing something
			if (control_valid(controls)) {
				// Run the parameter estimator
				DDParamEstimator_Step(&ddparamestimator_, state,
						controls, deltaT);
			}
		}
	}

	return updated;
}



/**
 * Push a new measurement in the buffer or the estimator.
 * This function is called from the CRTP when a pose measurement 
 * is received.
 */
bool estimatorDDEnqueuePose(const poseMeasurement_t *pos) {
	msg_counter = msg_counter + 1;

	// Measure the timestamp
	uint64_t timestamp = usecTimestamp(); // Time in microseconds
	float tstamp_s = timestamp / 1e6;

	// Extract Data from the pose
	float meas[DDEST_NUMOFINPUTS];

	float rpy[3];
	attitude_preprocessing(rpy, pos->quat);
	
	meas[0] = pos->x;
	meas[1] = pos->y;
	meas[2] = pos->z;
	meas[3] = (rpy[0]);
	meas[4] = (rpy[1]);
	meas[5] = (rpy[2]);

	// XXX I should protect this with mutex
	DDEstimator_AddMeas(&ddestimator_, meas, tstamp_s);

	return true;
}

DDParams estimatorDD_GetParam() {
	DDParams out = DDParamEstimator_GetParams(&ddparamestimator_);
	return out;
}



float estimatorDD_GetTMeasTimespan() {
	float out = 0.0;
	DDEstimator_GetMeasuresTimeInterval(&ddestimator_, &out);
	return out;
}






PARAM_GROUP_START(estimatorDD)
	PARAM_ADD(PARAM_FLOAT, Kest_x, &gains_x[0])
	PARAM_ADD(PARAM_FLOAT, Kest_x_d, &gains_x[1])
	PARAM_ADD(PARAM_FLOAT, Kest_y, &gains_y[0])
	PARAM_ADD(PARAM_FLOAT, Kest_y_d, &gains_y[1])

	PARAM_ADD(PARAM_FLOAT, Kest_2d0, &gains_2d[0])
	PARAM_ADD(PARAM_FLOAT, Kest_2d1, &gains_2d[1])
	PARAM_ADD(PARAM_FLOAT, Kest_2d2, &gains_2d[2])
	PARAM_ADD(PARAM_FLOAT, Kest_2d3, &gains_2d[3])
	PARAM_ADD(PARAM_FLOAT, Kest_2d4, &gains_2d[4])
	PARAM_ADD(PARAM_FLOAT, Kest_2d5, &gains_2d[5])
	PARAM_ADD(PARAM_FLOAT, Kest_2d6, &gains_2d[6])
	PARAM_ADD(PARAM_FLOAT, Kest_2d7, &gains_2d[7])
	PARAM_ADD(PARAM_FLOAT, Kest_2d8, &gains_2d[8])
	PARAM_ADD(PARAM_FLOAT, Kest_2d9, &gains_2d[9])
	PARAM_ADD(PARAM_FLOAT, Kest_2d10, &gains_2d[10])
	PARAM_ADD(PARAM_FLOAT, Kest_2d11, &gains_2d[11])
	PARAM_ADD(PARAM_FLOAT, Kest_2d12, &gains_2d[12])
	PARAM_ADD(PARAM_FLOAT, Kest_2d13, &gains_2d[13])
	PARAM_ADD(PARAM_FLOAT, Kest_2d14, &gains_2d[14])
	PARAM_ADD(PARAM_FLOAT, Kest_2d15, &gains_2d[15])
	PARAM_ADD(PARAM_FLOAT, Kest_2d16, &gains_2d[16])
	PARAM_ADD(PARAM_FLOAT, Kest_2d17, &gains_2d[17])
	PARAM_ADD(PARAM_FLOAT, Kest_2d18, &gains_2d[18])
	PARAM_ADD(PARAM_FLOAT, Kest_2d19, &gains_2d[19])
PARAM_GROUP_STOP(estimatorDD)



LOG_GROUP_START(estimator_dd_states)
	LOG_ADD(LOG_FLOAT, xx, &state_vec[DDEST_X])
	LOG_ADD(LOG_FLOAT, yy, &state_vec[DDEST_Y])
	LOG_ADD(LOG_FLOAT, zz, &state_vec[DDEST_Z])
	LOG_ADD(LOG_FLOAT, roll, &state_vec[DDEST_ROLL])
	LOG_ADD(LOG_FLOAT, pitch, &state_vec[DDEST_PITCH])
	LOG_ADD(LOG_FLOAT, yaw, &state_vec[DDEST_YAW])
LOG_GROUP_STOP(estimator_dd_states)
