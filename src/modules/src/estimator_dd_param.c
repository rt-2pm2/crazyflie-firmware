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
#include "estimator_dd_param.h"
#include "debug.h"




// ===========================================================
//			DATA BUFFERS
// ===========================================================


// ===========================================================

//
// DDESTIMATOR1D DATA STRUCTURE
//
void DDParamEstimator1D_Init(DDParamEstimator1D* pe) {
	pe->alpha = 0.0;
	pe->beta = 0.0;
	pe->initialized = true;
}

void DDParamEstimator1D_Step(DDParamEstimator1D* pe,
		float state_acc, float input, float T) {
	// Get the current value from the global variables
	float alpha = pe->alpha;
	float beta = pe->beta;
	
	// Local variables
	float alpha_new = 0.0f;
	float beta_new = 0.0f;

	float sqrt_T = sqrtf(T);

	//Compute Updates
	alpha_new = alpha +
		pe->est_gains[0] * sqrt_T * (state_acc -  (alpha + beta * input));
	beta_new = beta +
		pe->est_gains[1] * sqrt_T * input * (state_acc - (alpha + beta * input));
	
	
	if (beta_new < pe->beta_bounds[0]) {
		beta_new = pe->beta_bounds[0];
		//DEBUG_PRINT("Input 14 [ %.3f, %.3f]\n",
		//  (double)alpha_new, (double)beta_new);
	}

	if (beta_new > pe->beta_bounds[1]) {
		beta_new = pe->beta_bounds[1];
		//DEBUG_PRINT("Input 15 [ %.3f, %.3f]\n",
		//(double)alpha_new, (double)beta_new);
	}
	
	//Update global variables
	pe->alpha = alpha_new;
	pe->beta = beta_new;
}

void DDParamEstimator1D_SetGains(DDParamEstimator1D* pe,
		const float gains[2]) {
	int i = 0;
	for (i = 0; i < 2; i++) {
		pe->est_gains[i] = gains[i];
	}
}

void DDParamEstimator1D_SetBetaBounds(DDParamEstimator1D* pe, const float bbounds[2]) {
	int i = 0;
	for (i = 0; i < 2; i++) {
		pe->beta_bounds[i] = bbounds[i];
	}
}


void DDParamEstimator1D_GetParams(DDParamEstimator1D* pe, float* alpha, float* beta) {
	*alpha = pe->alpha;
	*beta = pe->beta;
} 

void DDParamEstimator1D_SetParams(DDParamEstimator1D* pe, float alpha, float beta) {
	pe->alpha = alpha;
	pe->beta = beta;
}



//
// DDESTIMATOR2D DATA STRUCTURE
//
void DDParamEstimator2D_Init(DDParamEstimator2D* pe) {
	// Initialized the data structures.
	arm_mat_init_f32(&pe->Am,
			DDESTPAR_STATE2DSIZE, 1,
			pe->alpha);
	arm_mat_init_f32(&pe->Bm,
			DDESTPAR_STATE2DSIZE, DDESTPAR_INPUT2DSIZE,
			pe->beta);
	arm_mat_init_f32(&pe->WeightedInputsm,
			DDESTPAR_STATE2DSIZE, 1,
			pe->WeightedInputs);
	arm_mat_init_f32(&pe->EstimatedAccm,
			DDESTPAR_STATE2DSIZE, 1,
			pe->EstimatedAcc);
	arm_mat_init_f32(&pe->ErrorAccm,
		DDESTPAR_STATE2DSIZE, 1,
		pe->ErrorAcc);

	pe->initialized = true;
}

void DDParamEstimator2D_Reset(DDParamEstimator2D* pe) {
	DDParamEstimator2D_Init(pe);
}

void DDParamEstimator2D_Step(DDParamEstimator2D* pe,
		float input[DDESTPAR_INPUT2DSIZE],
		float state_acc[DDESTPAR_STATE2DSIZE], float deltaT) {

	// Get the current value from the global variables
	float alpha[DDESTPAR_INPUT2DSIZE];
	float beta[DDESTPAR_BETA2DSIZE];
	int counter;

	// Copy the parameters in local arrays
	for (counter=0; counter < DDESTPAR_INPUT2DSIZE; counter++) {
		alpha[counter] = pe->alpha[counter];
	}
	for (counter = 0; counter < DDESTPAR_BETA2DSIZE; counter++) {
		beta[counter] = pe->beta[counter];
	}
	
	// Create Arm handler for the input and state vector
	arm_matrix_instance_f32 Statem;
	arm_mat_init_f32(&Statem,
		DDESTPAR_STATE2DSIZE, 1,
		state_acc);

        arm_matrix_instance_f32 Inputm;
	arm_mat_init_f32(&Inputm,
		DDESTPAR_INPUT2DSIZE, 1,
		input);
	
	// Compute Update
	arm_mat_mult_f32(&pe->Bm, &Inputm, &pe->WeightedInputsm);
	arm_mat_add_f32(&pe->Am, &pe->WeightedInputsm, &pe->EstimatedAccm);
	arm_mat_sub_f32(&Statem, &pe->EstimatedAccm, &pe->ErrorAccm);
	
	
	// Allocate temporary variable to store the update value of the 
	// parameters.
	float alpha_new[DDESTPAR_ALPHA2DSIZE];
	float beta_new[DDESTPAR_BETA2DSIZE];

	float sqrt_T = sqrtf(deltaT);
	float phi[5]={1, input[0], input[1], input[2], input[3]};
	// Update the parameters
	for(counter = 0; counter < DDESTPAR_ALPHA2DSIZE; counter++) {
		float error_i = pe->ErrorAcc[counter];
		// Alphas
		alpha_new[counter] = alpha[counter] + pe->est_gains[counter*DDESTPAR_ALPHA2DSIZE] * phi[0] * sqrt_T * error_i;
		// Betas	
		beta_new[counter*DDESTPAR_ALPHA2DSIZE] = 
			beta[counter*DDESTPAR_ALPHA2DSIZE] + pe->est_gains[1 + counter * DDESTPAR_ALPHA2DSIZE] * phi[1] * sqrt_T * error_i;
		beta_new[counter*DDESTPAR_ALPHA2DSIZE + 1] =
			beta[counter*DDESTPAR_ALPHA2DSIZE + 1] + pe->est_gains[2 + counter*DDESTPAR_ALPHA2DSIZE] * phi[2] * sqrt_T * error_i;
		beta_new[counter*DDESTPAR_ALPHA2DSIZE + 2] =
			beta[counter*DDESTPAR_ALPHA2DSIZE + 2] + pe->est_gains[3 + counter*DDESTPAR_ALPHA2DSIZE] * phi[3] * sqrt_T * error_i;
		beta_new[counter*DDESTPAR_ALPHA2DSIZE + 3] =
			beta[counter*DDESTPAR_ALPHA2DSIZE + 3] + pe->est_gains[4 + counter*DDESTPAR_ALPHA2DSIZE] * phi[4] * sqrt_T * error_i;
	}
	
	// Compute Projections
	for(counter=0; counter< DDESTPAR_BETA2DSIZE; counter++){
		if (beta_new[counter] < pe->beta_lbounds[counter]) {
			beta_new[counter] = pe->beta_lbounds[counter];
			//DEBUG_PRINT("Input 14 [ %.3f, %.3f]\n", (double)alpha_new, (double)beta_new);
		}
		if (beta_new[counter] > pe->beta_ubounds[counter]) {
			beta_new[counter] = pe->beta_ubounds[counter];
			//DEBUG_PRINT("Input 14 [ %.3f, %.3f]\n", (double)alpha_new, (double)beta_new);
		}
	}
	
	// Update global variables
	for(counter = 0; counter < DDESTPAR_ALPHA2DSIZE; counter++){
		pe->alpha[counter] = alpha_new[counter];
	}

	for(counter = 0; counter < DDESTPAR_BETA2DSIZE; counter++){
		pe->beta[counter] = beta_new[counter];
	}
}

void DDParamEstimator2D_SetGains(DDParamEstimator2D* pe,
		const float gains[DDESTPAR_GAINS2DSIZE]) {
	int i = 0;
	for (i = 0; i < DDESTPAR_GAINS2DSIZE; i++) {
		pe->est_gains[i] = gains[i];
	}
}

void DDParamEstimator2D_GetParams(DDParamEstimator2D* pe,
		float alpha[DDESTPAR_ALPHA2DSIZE],
		float beta[DDESTPAR_BETA2DSIZE]) {
	int i;

	for (i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
		alpha[i] = pe->alpha[i];
	}

	for (i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
		beta[i] = pe->beta[i];
	}
}

void DDParamEstimator2D_SetParams(DDParamEstimator2D* pe,
		const float alpha[DDESTPAR_ALPHA2DSIZE],
		const float beta[DDESTPAR_BETA2DSIZE]) {
	int i;
	for (i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
		pe->alpha[i] = alpha[i];
	}

	for (i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
		pe->beta[i] = beta[i];
	}
}




// 
// DDPARAMESTIMATOR DATA STRUCTURE
//
void DDParamEstimator_Init(DDParamEstimator* pe) {
	int i;
	for (i = 0; i < 2; i++) {
		DDParamEstimator1D_Init(&pe->paramest1D[i]);
	}
	DDParamEstimator2D_Init(&pe->paramest2D);

	pe->params.alpha2dsize = DDESTPAR_ALPHA2DSIZE;
	pe->params.beta2dsize = DDESTPAR_BETA2DSIZE;

	pe->initialized = true;
	pe->params.valid = false;
}

void DDParamEstimator_Reset(DDParamEstimator* pe) {
	DDParamEstimator_Init(pe);
}



/** 
 * Estimation step
 */
void DDParamEstimator_Step(DDParamEstimator* pe, state_t* ps,
		const float input[DDEST_NUMOFINPUTS],
		float deltaT) {
	if (pe->initialized) {
		//XXX Ask Lucas for the inputs

		// Estimate the paremeters on X
		float state_accx = ps->acc.x;
		float input_x = ps->attitude.pitch; 
		DDParamEstimator1D_Step(&pe->paramest1D[0],
				state_accx,
				input_x, deltaT);

		// Estimate the paremeters on Y
		float state_accy = ps->acc.y;
		float input_y = ps->attitude.roll;
		DDParamEstimator1D_Step(&pe->paramest1D[1],
				state_accy,
				input_y, deltaT);


		// Estimate the paremeters on the rest
		float state_acczatt[DDESTPAR_STATE2DSIZE];
		state_acczatt[0] = ps->acc.z;
		state_acczatt[1] = ps->attitudeAcc.roll;; 
		state_acczatt[2] = ps->attitudeAcc.pitch;
		state_acczatt[3] = ps->attitudeAcc.yaw;

		float input_zatt[4];
		DDParamEstimator2D_Step(&pe->paramest2D,
				state_acczatt,
				input_zatt, deltaT);

		// Update the internal structure
		pe->params.alpha_x = pe->paramest1D[0].alpha;
		pe->params.beta_x = pe->paramest1D[0].beta;

		pe->params.alpha_y = pe->paramest1D[1].alpha;
		pe->params.beta_y = pe->paramest1D[1].beta;

		for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
			pe->params.alpha2d[i] = pe->paramest2D.alpha[i];
		}

		for (int i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
			pe->params.beta2d[i] = pe->paramest2D.beta[i];
		}
		pe->params.valid = true;

	} else {
		DEBUG_PRINT("Param Estimator not initialized!");
		DDParamEstimator_Init(pe);
	}
}

void DDParamEstimator_SetGains(DDParamEstimator* pe,
		float gains_x[2], float gains_y[2],
		float gains_2d[DDESTPAR_GAINS2DSIZE]) {

	DDParamEstimator1D_SetGains(&pe->paramest1D[0], gains_x);
	DDParamEstimator1D_SetGains(&pe->paramest1D[1], gains_y);

	DDParamEstimator2D_SetGains(&pe->paramest2D, gains_2d);
}


DDParams DDParamEstimator_GetParams(DDParamEstimator* pe) {
	return pe->params;
}

