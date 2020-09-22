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

#include <math.h>

#include "math3d.h"
#include "estimator_dd_objects.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"


// ===========================================================
//			DATA BUFFERS
// ===========================================================


// ===========================================================
// DDMEAS DATA STRUCTURE
//
void DDMeas_Init(DDMeas* pm) {
	int i;
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		pm->meas[i] = 0;
		pm->sampleT[i] = 0;
	}
	pm->Ybuffm.numRows = DDEST_BUFFERSIZE;
	pm->Ybuffm.numCols = 1;
	pm->Ybuffm.pData = pm->meas;
	pm->nmeas = 0;
}

void DDMeas_AddMeas(DDMeas* pm, float m, float stamp) {
	int index = 0;
for (index = 1; index < DDEST_BUFFERSIZE; index++) {
		pm->meas[DDEST_BUFFERSIZE-index] = 
			pm->meas[DDEST_BUFFERSIZE-index-1];
		pm->sampleT[DDEST_BUFFERSIZE-index] =
			pm->sampleT[DDEST_BUFFERSIZE-index-1];
	}

	pm->meas[0] = m;
	pm->sampleT[0] = stamp;
	pm->nmeas++;
}

void DDMeas_GetMeas(DDMeas* pm, float m[DDEST_BUFFERSIZE],
		float t[DDEST_BUFFERSIZE]) {
	int i = 0;	
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		m[i] = pm->meas[i];
		t[i] = pm->sampleT[i];
	}
}

void DDMeas_GetTimestamps(DDMeas* pm, float tstamp[DDEST_BUFFERSIZE]) {
	int i = 0;	
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		tstamp[i] = pm->sampleT[i];
	}
}


// 
// DDOBS DATA STRUCTURE
//
// =============================================================
// PRIVATE STUFF
void DDObs_EvalPseudoInv(DDObs* po) {
	arm_status bad;
	// Temp Buffers for the evaluation of the pseudoinverse
	float TempNxNy[DDEST_STATESIZE1D * DDEST_BUFFERSIZE];
	float TempNxNx[DDEST_STATESIZE1D * DDEST_STATESIZE1D];
	float TempNxNx2[DDEST_STATESIZE1D * DDEST_STATESIZE1D];

	// to Temporary Objects
	arm_matrix_instance_f32 TempNxNym = {
		DDEST_STATESIZE1D, DDEST_BUFFERSIZE, TempNxNy};
	arm_matrix_instance_f32 TempNxNxm = {
		DDEST_STATESIZE1D, DDEST_STATESIZE1D, TempNxNx};
	arm_matrix_instance_f32 TempNxNx2m = {
		DDEST_STATESIZE1D, DDEST_STATESIZE1D, TempNxNx2};

	

	// O'
	bad = arm_mat_trans_f32(&(po->Obsm), &TempNxNym);
	if (bad) {DEBUG_PRINT("Error Transpose: %d \n", bad);}

	// (O' x O)
	bad = arm_mat_mult_f32(&TempNxNym, &po->Obsm, &TempNxNxm);
	if (bad) {DEBUG_PRINT("Error Multiply: %d \n", bad);}

	// (O' x O)^-1 x O' = Pseudo inverse
	bad = arm_mat_inverse_f32(&TempNxNxm, &TempNxNx2m);
	if (bad) {DEBUG_PRINT("Error Inversion: %d \n", bad);}

	bad = arm_mat_mult_f32(&TempNxNx2m, &TempNxNym, &po->Obs_invm);
	if (bad) {DEBUG_PRINT("Error Multiply: %d \n", bad);}	
}

// PUBLIC STUFF
void DDObs_Init(DDObs* po) {
	int i;

	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		po->deltaT[i] = 0;
	}

	for (i = 0; i < DDEST_BUFFERSIZE * DDEST_STATESIZE1D; i++) {
		po->Obs[i] = 0;
		po->Obs_inv[i] = 0;
	}
	
	po->Obsm.numRows = DDEST_BUFFERSIZE;
	po->Obsm.numCols = DDEST_STATESIZE1D;
	po->Obsm.pData = po->Obs;
	/*
	arm_mat_init_f32(
			&po->Obsm,
			DDEST_BUFFERSIZE,
			DDEST_STATESIZE1D,
			&po->Obs[0]
			);
	*/

	po->Obs_invm.numRows = DDEST_STATESIZE1D;
	po->Obs_invm.numCols = DDEST_BUFFERSIZE;
	po->Obs_invm.pData = po->Obs_inv;

	po->initialized = true;
}

void DDObs_Update(DDObs* po, const float tstamps[DDEST_BUFFERSIZE]) {
	if (!po->initialized) {
		DEBUG_PRINT("DDObs not Initialized!\n");
		DDObs_Init(po);
	}

	int i;
	// Update the DeltaT vector in the structure
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		float dT = tstamps[0] - tstamps[i];
		po->deltaT[i] = dT;
	}

	static int counter = 0;
	if (counter++ % 1000 == 0) {
		DEBUG_PRINT("Time deltas: [");
		for (int i = 0; i < DDEST_BUFFERSIZE; i++) {
			DEBUG_PRINT("%1.4f ", (double)po->deltaT[i]);
		}
		DEBUG_PRINT("\n");
	}

	// Fill the Obs Matrix
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		float T = po->deltaT[i];
		po->Obs[i * DDEST_STATESIZE1D] = 1;
		po->Obs[(i * DDEST_STATESIZE1D) + 1] = -T;
		po->Obs[(i * DDEST_STATESIZE1D) + 2] = 0.5f * (T * T);
	}

	DDObs_EvalPseudoInv(po);
}

void DDObs_cpy(DDObs* dst, const DDObs* src) {
	int i;
	for (i = 0; i < DDEST_BUFFERSIZE; i++) {
		dst->deltaT[i] = src->deltaT[i];
	}
	
	for (i = 0; i < DDEST_BUFFERSIZE * DDEST_STATESIZE1D; i++) {
		dst->Obs[i] = src->Obs[i];
		dst->Obs_inv[i] = src->Obs_inv[i];
	}
}

void DDObs_GetMeasuresTimeInterval(DDObs* po, float* deltaT) {
	if (!po->initialized) {
		DEBUG_PRINT("DDObs not Initialized!\n");
		*deltaT = 0;
	}

	*deltaT = po->deltaT[DDEST_BUFFERSIZE - 1];
}



// 
// DDESTIMATOR1D DATA STRUCTURE
//
void DDEstimator1D_Init(DDEstimator1D* pe) {
	DDMeas_Init(&pe->meas_data);
	DDObs_Init(&pe->obs_data);

	pe->state_est_m.numRows = DDEST_STATESIZE1D;
	pe->state_est_m.numCols = 1;
	pe->state_est_m.pData = pe->state_est;

	pe->meas_counter = 0;

	pe->initialized = true;
}

void DDEstimator1D_Reset(DDEstimator1D* pe) {
	DDEstimator1D_Init(pe);
}

void DDEstimator1D_AddMeas(DDEstimator1D* pe, float m, float t) {
	if (!pe->initialized) {
		DDEstimator1D_Init(pe);
	}
	DDMeas_AddMeas(&pe->meas_data, m, t); 	
	pe->meas_counter++;
}

void DDEstimator1D_UpdateCtrl(DDEstimator1D* pe, float m) {
	pe->ctrl = m;
}

void DDEstimator1D_Step(DDEstimator1D* pe) {
	arm_mat_mult_f32(&pe->obs_data.Obs_invm, &pe->meas_data.Ybuffm,
			&pe->state_est_m);
}

void DDEstimator1D_GetState(DDEstimator1D* pe, float s[DDEST_STATESIZE1D]){
	int i;
	for (i = 0; i < DDEST_STATESIZE1D; i++) {
		s[i] = pe->state_est[i];
	}
}


// 
// DDESTIMATOR DATA STRUCTURE
//
void DDEstimator_Init(DDEstimator* pe) {
	int i;
	for (i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		DDEstimator1D_Init(&pe->estimators[i]);
	}
	DDObs_Init(&pe->obs_data);

	pe->sensors_mrt = 0.0;

	pe->msg_counter = 0;
	pe->ready = false;
	pe->initialized = true;
}

void DDEstimator_Reset(DDEstimator* pe) {
	DDEstimator_Init(pe);
}

void DDEstimator_AddMeas(DDEstimator* pe,
		const float m[DDEST_NUMOFCHANNELS],
		float tstamp) {

	if (!pe->initialized) {
		DEBUG_PRINT("DDEstimator Not Initialized!\n");
		DDEstimator_Init(pe);
	}

	int i;
	for (i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		DDEstimator1D_AddMeas(&pe->estimators[i], m[i], tstamp);
	}

	pe->msg_counter++;

	if (pe->msg_counter > DDEST_BUFFERSIZE) {
		pe->ready = true;
	}
}

void DDEstimator_UpdateCtrl(DDEstimator* pe, float m[DDEST_NUMOFINPUTS]) {
	for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		DDEstimator1D_UpdateCtrl(&pe->estimators[i], m[i]);
	}
}

/** 
 * Estimation step
 * The idea is that the measurements are filled in by the receiver thread. 
 * This step * is called by the another thread 
 * (maybe the controller thread).
 * The step take care of:
 * - Update the Observability matrix
 * - Estimate the state
 * - Estimate the parameters
 */
bool DDEstimator_Step(DDEstimator* pe) {
	if (pe->ready) {
		float timestamps[DDEST_BUFFERSIZE];

		// Take a pointer to the first channel just for 
		// getting the time vector
		DDMeas* pmeas = &pe->estimators[0].meas_data;
		DDMeas_GetTimestamps(pmeas, timestamps);

		float mrt = timestamps[0];
		if (mrt <= pe->sensors_mrt) {
			return false;
		}
		pe->sensors_mrt =  mrt;

		// Prepare for the estimation step 
		DDObs_Update(&pe->obs_data, timestamps);

		


		// Run the estimation on each channel
		for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
			DDEstimator1D* pest = &pe->estimators[i];

			DDObs_cpy(&pest->obs_data, &pe->obs_data);
			DDEstimator1D_Step(pest);
		}
	} else {
		return false;
	}

	return true;
}

void DDEstimator_GetMeasuresTimeInterval(DDEstimator* pe, float* deltaT) {
	DDObs_GetMeasuresTimeInterval(&pe->obs_data, deltaT);
}

void DDEstimator_GetState(DDEstimator* pe,
		float fullstate[DDEST_FULLSTATESIZE]) {
	int glob_index = 0;
	int i = 0;
	for (i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		float state1d[DDEST_STATESIZE1D];
		DDEstimator1D_GetState(&pe->estimators[i], state1d);
		int j = 0;
		for (j = 0; j < DDEST_STATESIZE1D; j++) {
			fullstate[glob_index++] = state1d[j];	
		}
	}
}

void DDEstimator_ExportState(DDEstimator* pe, state_t* ps) {
	for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		float state1d[DDEST_STATESIZE1D];
		DDEstimator1D_GetState(&pe->estimators[i], state1d);
		struct vec rpy = {0,0,0};
		uint32_t osTick = xTaskGetTickCount();
		switch (i) {
			case DDEST_XCHANNEL:
				ps->position.x = state1d[0];
				ps->velocity.x = state1d[1];
				ps->acc.x = state1d[2];
				break;
			case DDEST_YCHANNEL:
				ps->position.y = state1d[0];
				ps->velocity.y = state1d[1];
				ps->acc.y = state1d[2];
				break;
			case DDEST_ZCHANNEL:
				ps->position.z = state1d[0];
				ps->velocity.z = state1d[1];
				ps->acc.z = state1d[2];
				break;
			case DDEST_ROLLCHANNEL:
				ps->attitude.roll = state1d[0] * 180.0f / M_PI_F;
				ps->attitudeRate.roll = state1d[1] * 180.0f / M_PI_F;
				ps->attitudeAcc.roll = state1d[2] * 180.0f / M_PI_F;
				rpy.x = state1d[0];
				break;
			case DDEST_PITCHCHANNEL:
				ps->attitude.pitch = state1d[0] * 180.0f / M_PI_F;
				ps->attitudeRate.pitch = state1d[1] * 180.0f / M_PI_F;
				ps->attitudeAcc.pitch = state1d[2] * 180.0f / M_PI_F;
				rpy.y = state1d[0];
				break;
			case DDEST_YAWCHANNEL:
				ps->attitude.yaw = state1d[0] * 180.0f / M_PI_F;
				ps->attitudeRate.yaw = state1d[1] * 180.0f / M_PI_F;
				ps->attitudeAcc.yaw = state1d[2] * 180.0f / M_PI_F;
				rpy.z = state1d[0];
				break;
			default:
				DEBUG_PRINT("Something queer is going on here\n");
				break;
		}
		// Update the quaternion
		struct quat q = rpy2quat(rpy);
		ps->attitudeQuaternion.x = q.x;
		ps->attitudeQuaternion.y = q.y;
		ps->attitudeQuaternion.z = q.z;
		ps->attitudeQuaternion.w = q.w;

		// Update the timestamps
		ps->attitude.timestamp = osTick;
		ps->attitudeRate.timestamp = osTick;
		ps->attitudeAcc.timestamp = osTick;
		ps->attitudeQuaternion.timestamp = osTick;
		ps->position.timestamp = osTick;
		ps->velocity.timestamp = osTick;
		ps->acc.timestamp = osTick;
	}
}
