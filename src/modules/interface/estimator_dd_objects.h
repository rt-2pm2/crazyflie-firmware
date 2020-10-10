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
 *
 * estimatorDD_data.h - DataDriven Estimator Data Structures 
 *
 */
#ifndef __ESTIMATOR_DD_OBJECTS_H__
#define __ESTIMATOR_DD_OBJECTS_H__

#include "arm_math.h"
#include "stabilizer_types.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define ARM_MATH_MATRIX_CHECK

#define DDEST_BUFFERSIZE (5)
#define DDEST_NUMOFCHANNELS (6)
#define DDEST_NUMOFINPUTS (6)
#define DDEST_STATESIZE1D (3)
#define DDEST_FULLSTATESIZE ((DDEST_NUMOFCHANNELS)*(DDEST_STATESIZE1D))

#define DDEST_X (0)
#define DDEST_VX (1)
#define DDEST_AX (2)
#define DDEST_Y (3)
#define DDEST_VY (4)
#define DDEST_AY (5)
#define DDEST_Z (6)
#define DDEST_VZ (7)
#define DDEST_AZ (8)
#define DDEST_ROLL (9)
#define DDEST_VROLL (10)
#define DDEST_AROLL (11)
#define DDEST_PITCH (12)
#define DDEST_VPITCH (13)
#define DDEST_APITCH (14)
#define DDEST_YAW (15)
#define DDEST_VYAW (16)
#define DDEST_AYAW (17)

#define DDEST_XCHANNEL (0)
#define DDEST_YCHANNEL (1)
#define DDEST_ZCHANNEL (2)
#define DDEST_ROLLCHANNEL (3)
#define DDEST_PITCHCHANNEL (4)
#define DDEST_YAWCHANNEL (5)


typedef enum {
  fullState = 0,
  XYZState,
  AttitudeState,
} StateRequestType;


/**
 * Data structure for the measurements
 */
typedef struct DDMeas DDMeas;
struct DDMeas {
	// Vector of measurements
	float meas[DDEST_BUFFERSIZE];
	// Vector of timestamps (of the measurements)
	float sampleT[DDEST_BUFFERSIZE];
	
	// Arm datastructure associated with the measurements
	arm_matrix_instance_f32 Ybuffm;

	// Number of measurement in the buffer
	int nmeas;
};
// Public Methods
void DDMeas_Init(DDMeas* pm);
void DDMeas_AddMeas(DDMeas* pm, float measurement, float tstamp);
void DDMeas_GetMeas(DDMeas* pm, float measurements[DDEST_BUFFERSIZE],
		float tstamps[DDEST_BUFFERSIZE]);
void DDMeas_GetTimestamps(DDMeas* pm, float tstamps[DDEST_BUFFERSIZE]);


/**
 * Data structure for the observation matrix
 */
typedef struct DDObs DDObs;
struct DDObs {
	bool initialized;

	// Vector of DeltaT
	float deltaT[DDEST_BUFFERSIZE];
	// Observation Matrix
	// This buffer is updated using the timestamp of the measurements.
	float Obs[DDEST_BUFFERSIZE * DDEST_STATESIZE1D];
	// Inverse Observation Matrix
	// This is just memory buffer, the operations are made through the
	// arm_matrix.
	float Obs_inv[DDEST_STATESIZE1D * DDEST_BUFFERSIZE];

	// ARM Library Handlers
	// to the Obs matrix
	arm_matrix_instance_f32 Obsm;
	// to the Obs inverse matrix
	arm_matrix_instance_f32 Obs_invm;
	
};
// Public Methods
void DDObs_Init(DDObs* po);
/**
 * Update the Observation matrix given the vector
 * of deltaT.
 */
void DDObs_Update(DDObs* po, const float tstamps[DDEST_BUFFERSIZE]); 
void DDObs_cpy(DDObs* dst, const DDObs* src);
void DDObs_GetMeasuresTimeInterval(DDObs* po, float* deltaT);


/**
 * Estimator 1D data structure
 */
typedef struct DDEstimator1D DDEstimator1D;
struct DDEstimator1D {
	// Measurement Data
	DDMeas meas_data;
	uint64_t meas_counter;

	// Observation Data
	DDObs obs_data;

	// Data buffer for the status
	float state_est[DDEST_STATESIZE1D];
	// ARM data handler
	arm_matrix_instance_f32 state_est_m;

	float ctrl;

	bool ready;

	bool initialized;
};
// Public Methods
void DDEstimator1D_Init(DDEstimator1D* pe);
void DDEstimator1D_Reset(DDEstimator1D* pe);
void DDEstimator1D_AddMeas(DDEstimator1D* pe, float m, float t);
void DDEstimator1D_UpdateCtrl(DDEstimator1D* pe, float m);
void DDEstimator1D_Step(DDEstimator1D* pe);
void DDEstimator1D_GetState(DDEstimator1D* pe, float s[DDEST_STATESIZE1D]);


/**
 * Estimator Multi-D data structure
 */
typedef struct DDEstimator DDEstimator;
struct DDEstimator {
	SemaphoreHandle_t dataMutex;
 	StaticSemaphore_t dataMutexBuffer;

	// Measurement Data
	DDEstimator1D estimators[DDEST_NUMOFCHANNELS];

	// Observation support data
	// We are assuming to have a single source of measurements so
	// given the symmetry assumption on the channels, the Observation
	// matrix is the same. All the CHANNELS will use the same matrix, 
	// which will be computed only once.
	DDObs obs_data;

	uint64_t msg_counter;

	bool ready;

	float sensors_mrt;
	
	bool initialized;
};
// Public Methods
void DDEstimator_Init(DDEstimator* pe);
void DDEstimator_Reset(DDEstimator* pe);
void DDEstimator_AddMeas(DDEstimator* pe,
		const float m[DDEST_NUMOFCHANNELS],
		float tstamp);
void DDEstimator_UpdateCtrl(DDEstimator* pe,
		float m[DDEST_NUMOFINPUTS]);
bool DDEstimator_Step(DDEstimator* pe);
void DDEstimator_GetMeasuresTimeInterval(DDEstimator* pe, float* deltaT);
void DDEstimator_GetState(DDEstimator* pe,
		float fullstate[DDEST_FULLSTATESIZE]);
void DDEstimator_ExportState(DDEstimator* pe, state_t* state);



#endif //__ESTIMATOR_DD_DATA_H__
