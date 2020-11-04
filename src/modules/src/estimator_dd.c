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

#define CTRL_THRESHOLD (0.01f)

// STATIC VARIABLES

// Parameter Estimator Gains
static float gains_x[2] = {0.0001, 0.1};
static float gains_y[2] = {0.0001, 0.1};

static float gains_alpha2d[DDESTPAR_ALPHA2DSIZE] = {
	0.000002, 0.000002, 0.000002, 0.000002
};

static float gains_beta2d[DDESTPAR_BETA2DSIZE] = {
	0.001, 0.001, 0.001, 0.001,
	0.001, 0.001, 0.001, 0.001,
	0.001, 0.001, 0.001, 0.001,
	0.001, 0.001, 0.001, 0.001
};

// Initial Parameters
static float alpha_xy_init = 0;
static float beta_x_init = 10.0;
static float beta_y_init = -10.0;
static float alpha2d_init[DDESTPAR_ALPHA2DSIZE] = {-18.0, 0, 0, 0};
static float beta2d_init[DDESTPAR_BETA2DSIZE] = {
	5, 5, 5, 5,
	-50, -50, 50, 50,
	-50, 50, 50, -50,
	50, -50, 50, -50
};


// Bounds
static float beta_x_bounds[2] = {1.0, 18.0};
static float beta_y_bounds[2] = {-18.0, -1.0};
static float beta2d_lowerb[16] = {
	1,1,1,1,
	-100, -100, 1, 1,
	-100, 1, 1, -100,
	1, -100, 1, -100
};
static float beta2d_upperb[16] = {
	10,10,10,10,
	-1, -1, 100, 100,
	-1, 100, 100, -1,
	100, -1, 100, -1
};




// ==================================================================
//			GLOBALS
// Global counter of received sensor messages 
unsigned long int msg_counter = 0;
DDParams pp;

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


void estimatorDDSetparams(float ax, float bx, float ay, float by, 
		float a2d[DDESTPAR_ALPHA2DSIZE],
		float b2d[DDESTPAR_BETA2DSIZE]) {

	DDParams par; 
	par.valid = false; // Send the parameters with the false flag
	
	par.alpha_x = ax;
	par.alpha_y = ay;
	
	par.beta_x = bx;
	par.beta_y = by;

	for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
		 par.alpha2d[i] = a2d[i];
	}

	for (int i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
		 par.beta2d[i] = b2d[i];
	}

	par.alpha2dsize = DDESTPAR_ALPHA2DSIZE;
	par.beta2dsize = DDESTPAR_BETA2DSIZE; 

	DDParamEstimator_SetParams(&ddparamestimator_, par);
}

// ===================================================================
// 				DDEstimator



// ==================================================================
// 				PRIVATE
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
	
	// Initialize the parameters
	estimatorDDSetparams(
			alpha_xy_init,
			beta_x_init,
			alpha_xy_init,
			beta_y_init, 
			alpha2d_init,
			beta2d_init
			);

	// Initialize the bounds
	DDParamEstimator_SetBounds(&ddparamestimator_, 
		beta_x_bounds, beta_y_bounds,
		beta2d_lowerb, beta2d_upperb);

	// Initialize the gains
	DDParamEstimator_SetGains(&ddparamestimator_,
			gains_x, gains_y,
			gains_alpha2d,
			gains_beta2d);
}

bool estimatorDDTest(void) {
	return true;
}

void estimatorDD(state_t *state,
		sensorData_t *sensors,
		control_t *control,
		const uint32_t tick) {
	if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
		return;
	}
	DDEstimator_GetState(&ddestimator_, state_vec);
}


bool estimatorDD_Step(state_t *state,
		const float controls[4],
		const uint32_t tick) {
	bool updated = false;

	// Update the gains
	DDParamEstimator_SetGains(&ddparamestimator_, gains_x,
			gains_y, gains_alpha2d, gains_beta2d);

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
			pp = DDParamEstimator_GetParams(&ddparamestimator_);
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



PARAM_GROUP_START(estimatorDD_par)
	PARAM_ADD(PARAM_FLOAT, K_x, &gains_x[0])
	PARAM_ADD(PARAM_FLOAT, K_x_d, &gains_x[1])
	PARAM_ADD(PARAM_FLOAT, K_y, &gains_y[0])
	PARAM_ADD(PARAM_FLOAT, K_y_d, &gains_y[1])
	PARAM_ADD(PARAM_FLOAT, K_a2d0, &gains_alpha2d[0])
	PARAM_ADD(PARAM_FLOAT, K_a2d1, &gains_alpha2d[1])
	PARAM_ADD(PARAM_FLOAT, K_a2d2, &gains_alpha2d[2])
	PARAM_ADD(PARAM_FLOAT, K_a2d3, &gains_alpha2d[3])
	//
	PARAM_ADD(PARAM_FLOAT, K_b2d0, &gains_beta2d[0])
	PARAM_ADD(PARAM_FLOAT, K_b2d1, &gains_beta2d[1])
	PARAM_ADD(PARAM_FLOAT, K_b2d2, &gains_beta2d[2])
	PARAM_ADD(PARAM_FLOAT, K_b2d3, &gains_beta2d[3])
	PARAM_ADD(PARAM_FLOAT, K_b2d4, &gains_beta2d[4])
	PARAM_ADD(PARAM_FLOAT, K_b2d5, &gains_beta2d[5])
	PARAM_ADD(PARAM_FLOAT, K_b2d6, &gains_beta2d[6])
	PARAM_ADD(PARAM_FLOAT, K_b2d7, &gains_beta2d[7])
	PARAM_ADD(PARAM_FLOAT, K_b2d8, &gains_beta2d[8])
	PARAM_ADD(PARAM_FLOAT, K_b2d9, &gains_beta2d[9])
	PARAM_ADD(PARAM_FLOAT, K_b2d10, &gains_beta2d[10])
	PARAM_ADD(PARAM_FLOAT, K_b2d11, &gains_beta2d[11])
	PARAM_ADD(PARAM_FLOAT, K_b2d12, &gains_beta2d[12])
	PARAM_ADD(PARAM_FLOAT, K_b2d13, &gains_beta2d[13])
	PARAM_ADD(PARAM_FLOAT, K_b2d14, &gains_beta2d[14])
	PARAM_ADD(PARAM_FLOAT, K_b2d15, &gains_beta2d[15])
PARAM_GROUP_STOP(estimatorDD_par)


LOG_GROUP_START(estimatorDD_log)
	LOG_ADD(LOG_FLOAT, xx, &state_vec[DDEST_X])
	LOG_ADD(LOG_FLOAT, yy, &state_vec[DDEST_Y])
	LOG_ADD(LOG_FLOAT, zz, &state_vec[DDEST_Z])
	LOG_ADD(LOG_FLOAT, roll, &state_vec[DDEST_ROLL])
	LOG_ADD(LOG_FLOAT, pitch, &state_vec[DDEST_PITCH])
	LOG_ADD(LOG_FLOAT, yaw, &state_vec[DDEST_YAW])
LOG_GROUP_STOP(estimatorDD_log)

LOG_GROUP_START(estParamDD_log)
	LOG_ADD(LOG_FLOAT, alpha_x, &pp.alpha_x)
	LOG_ADD(LOG_FLOAT, alpha_y, &pp.alpha_y)
	LOG_ADD(LOG_FLOAT, beta_x, &pp.beta_x)
	LOG_ADD(LOG_FLOAT, beta_y, &pp.beta_y)
LOG_GROUP_STOP(estParamDD_log)