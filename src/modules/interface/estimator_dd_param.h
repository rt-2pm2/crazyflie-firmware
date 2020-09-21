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
#ifndef __ESTIMATOR_DD_PARAM_H__
#define __ESTIMATOR_DD_PARAM_H__


#include "arm_math.h"

#include "estimator_dd_objects.h"

#define ARM_MATH_MATRIX_CHECK

#define DDESTPAR_STATE1DSIZE (3)

#define DDESTPAR_STATE2DSIZE (4)
#define DDESTPAR_INPUT2DSIZE (4)
#define DDESTPAR_ALPHA2DSIZE (DDESTPAR_INPUT2DSIZE)
#define DDESTPAR_BETA2DSIZE (DDESTPAR_INPUT2DSIZE * DDESTPAR_INPUT2DSIZE)
#define DDESTPAR_GAINS2DSIZE (DDESTPAR_ALPHA2DSIZE + DDESTPAR_BETA2DSIZE)


/**
 * Data structure for a better export 
 * of the paramters
 */
typedef struct DDParams_s DDParams;
struct DDParams_s {
	bool valid;
	
	float alpha_x;
	float alpha_y;
	
	float beta_x;
	float beta_y;

	float alpha2d[DDESTPAR_ALPHA2DSIZE];
	float beta2d[DDESTPAR_BETA2DSIZE];

	size_t alpha2dsize;
	size_t beta2dsize; 
};


/**
 * Data structures for the parameters 
 */
typedef struct DDParamEstimator1D DDParamEstimator1D;
struct DDParamEstimator1D {
	float alpha;
	float beta;

	// Estimator Gains
	float est_gains[2];

	// Bounds on the beta parameter
	float beta_bounds[2];

	bool initialized;
};
// Public Methods
void DDParamEstimator1D_Init(DDParamEstimator1D* pm);
void DDParamEstimator1D_Step(DDParamEstimator1D* pm,
		float state_acc,float input, float T);
void DDParamEstimator1D_SetGains(DDParamEstimator1D* pm,
		const float gains[2]);
void DDParamEstimator1D_SetBetaBounds(DDParamEstimator1D* pm,
		const float bbounds[2]);
void DDParamEstimator1D_GetParams(DDParamEstimator1D* pm,
		float* alpha, float* beta);
void DDParamEstimator1D_SetParams(DDParamEstimator1D* pm,
		float alpha, float beta);



/**
 * Data structure for the parameters 
 */
typedef struct DDParamEstimator2D DDParamEstimator2D;
struct DDParamEstimator2D {
	float alpha[DDESTPAR_ALPHA2DSIZE];
	float beta[DDESTPAR_BETA2DSIZE];

	float est_gains[DDESTPAR_GAINS2DSIZE];

	float beta_lbounds[DDESTPAR_BETA2DSIZE];
	float beta_ubounds[DDESTPAR_BETA2DSIZE];

	// Buffers for matrix operations
	float ErrorAcc[DDESTPAR_STATE2DSIZE];
	float EstimatedAcc[DDESTPAR_STATE2DSIZE];
	float WeightedInputs[DDESTPAR_STATE2DSIZE];

	// Arm Matrix Handlers
	arm_matrix_instance_f32 Am;
	arm_matrix_instance_f32 Bm;
        arm_matrix_instance_f32 WeightedInputsm;
        arm_matrix_instance_f32 EstimatedAccm;
        arm_matrix_instance_f32 ErrorAccm;

	bool initialized;
};
// Public Methods
void DDParamEstimator2D_Init(DDParamEstimator2D* pm);
void DDParamEstimator2D_Reset(DDParamEstimator2D* pm);
void DDParamEstimator2D_Step(DDParamEstimator2D* pm,
		float input[DDESTPAR_INPUT2DSIZE],
		float state[DDESTPAR_STATE2DSIZE], float T);
void DDParamEstimator2D_SetGains(DDParamEstimator2D* pm,
		const float gains[DDESTPAR_GAINS2DSIZE]);
void DDParamEstimator2D_GetParams(DDParamEstimator2D* pm,
		float alpha[DDESTPAR_ALPHA2DSIZE],
		float beta[DDESTPAR_BETA2DSIZE]);
void DDParamEstimator2D_SetParams(DDParamEstimator2D* pm,
		const float alpha[DDESTPAR_ALPHA2DSIZE],
		const float beta[DDESTPAR_BETA2DSIZE]);

/**
 * Parameter Estimator data structure
 */
typedef struct DDParamEstimator DDParamEstimator;
struct DDParamEstimator {
	// Measurement Data
	DDParamEstimator1D paramest1D[2];
	DDParamEstimator2D paramest2D; 

	DDParams params;

	bool initialized;
};
// Public Methods
/**
 * Reset and init function of the parameter estimator
 */
void DDParamEstimator_Init(DDParamEstimator* pe);
void DDParamEstimator_Reset(DDParamEstimator* pe);
/**
 * Perform a estimation step
 */
void DDParamEstimator_Step(DDParamEstimator* pe,
		state_t* ps,
		const float input[DDEST_NUMOFINPUTS],
		float deltaT
		);

/**
 * Set the parameter estimators gains
 */
void DDParamEstimator_SetGains(DDParamEstimator* pe,
		float gains_x[2],
		float gains_y[2],
		float gains_2d[DDESTPAR_GAINS2DSIZE]
		);


/**
 * Get the data structure of the parameters from the estimator
 */
DDParams DDParamEstimator_GetParams(DDParamEstimator* pe);

/**
 * Get the data structure of the parameters from the estimator
 */
void DDParamEstimator_SetParams(DDParamEstimator* pe, DDParams pa);

/**
 * Set the bound for the beta parameters
 */
void DDParamEstimator_SetBounds(DDParamEstimator* pe, 
		float beta_x[2], float beta_y[2],
		float beta2dlb[DDESTPAR_BETA2DSIZE],
		float beta2dup[DDESTPAR_BETA2DSIZE]);

#endif //__ESTIMATOR_DD_DATA_H__
