/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_dd_objects.h - DataDriven Controller Interface
 */

#include <math.h>
#include "math3d.h"
#include "stabilizer_types.h"
#include "controller_dd_objects.h"
#include "debug.h"
#include "log.h"

#define MAXTILT (3.0f * M_PI_F / 8.0f)
#define MAXANGULARSPEED (2000.0f)
#define MAXANGULARACC (50000.0f)


// PRIVATE
float phat_log[4];

// Limit values
float boundval(float input, float llim, float ulim) {
	return fmaxf(fminf(input, ulim), llim);
}

// Split the setpoint in different components
void setpoint2arrays(const setpoint_t* sp,
		float xsp[2],
		float ysp[2],
		float zsp[2],
		float yawsp[2]) {

	xsp[0] = sp->position.x;
	xsp[1] = sp->velocity.x;

	ysp[0] = sp->position.y;
	ysp[1] = sp->velocity.y;

	zsp[0] = sp->position.z;
	zsp[1] = sp->velocity.z;

	yawsp[0] = sp->attitude.yaw / 180 * M_PI_F;
	yawsp[1] = sp->attitudeRate.yaw / 180 * M_PI_F;
}

// Split the state estimate structure in different components
void state2arrays(const state_t* sp,
		float xest[3],
		float yest[3],
		float zest[3],
		float rollest[3],
		float pitchest[3],
		float yawest[3]) {

	xest[0] = sp->position.x;
	xest[1] = sp->velocity.x;
	xest[2] = sp->acc.x;


	yest[0] = sp->position.y;
	yest[1] = sp->velocity.y;
	yest[2] = sp->acc.y;

	zest[0] = sp->position.z;
	zest[1] = sp->velocity.z;
	zest[2] = sp->acc.z;

	rollest[0] = sp->attitude.roll / 180.0f * M_PI_F;
	rollest[1] = boundval(sp->attitudeRate.roll / 180.0f * M_PI_F,
			-MAXANGULARSPEED,
			MAXANGULARSPEED);
	rollest[2] = boundval(sp->attitudeAcc.roll / 180.0f * M_PI_F,
			-MAXANGULARACC,
			MAXANGULARACC);

	pitchest[0] = sp->attitude.pitch / 180.0f * M_PI_F;
	pitchest[1] = boundval(sp->attitudeRate.pitch / 180.0f * M_PI_F,
			-MAXANGULARSPEED,
			MAXANGULARSPEED);
	pitchest[2] = boundval(sp->attitudeAcc.pitch / 180.0f * M_PI_F,
			-MAXANGULARACC,
			MAXANGULARACC);

	yawest[0] = sp->attitude.yaw / 180.0f * M_PI_F;
	yawest[1] = boundval(sp->attitudeRate.yaw / 180.0f * M_PI_F,
			-MAXANGULARSPEED,
			MAXANGULARSPEED);
	yawest[2] = boundval(sp->attitudeAcc.yaw / 180.0f * M_PI_F,
			-MAXANGULARACC,
			MAXANGULARACC);
}

void eval_pseudoinv(arm_matrix_instance_f32* dest, arm_matrix_instance_f32* src) {
	arm_status bad;
	// Temp Buffers for the evaluation of the pseudoinverse
	float TempNxNy[DDESTPAR_BETA2DSIZE];
	float TempNxNx[DDESTPAR_BETA2DSIZE];
	float TempNxNx2[DDESTPAR_BETA2DSIZE];

	// to Temporary Objects
	arm_matrix_instance_f32 TempNxNym = {
		DDCTRL_OUTPUTSIZE, DDCTRL_OUTPUTSIZE, TempNxNy};
	arm_matrix_instance_f32 TempNxNxm = {
		DDCTRL_OUTPUTSIZE, DDCTRL_OUTPUTSIZE, TempNxNx};
	arm_matrix_instance_f32 TempNxNx2m = {
		DDCTRL_OUTPUTSIZE, DDCTRL_OUTPUTSIZE, TempNxNx2};

	// O'
	bad = arm_mat_trans_f32(src, &TempNxNym);
	if (bad) {DEBUG_PRINT("Error Transpose: %d \n", bad);}

	// (O' x O)
	bad = arm_mat_mult_f32(&TempNxNym, src, &TempNxNxm);
	if (bad) {DEBUG_PRINT("Error Multiply: %d \n", bad);}

	// (O' x O)^-1 x O' = Pseudo inverse
	bad = arm_mat_inverse_f32(&TempNxNxm, &TempNxNx2m);
	if (bad) {DEBUG_PRINT("Error Inversion: %d \n", bad);}

	bad = arm_mat_mult_f32(&TempNxNx2m, &TempNxNym, dest);
	if (bad) {DEBUG_PRINT("Error Multiply: %d \n", bad);}	
}




// PUBLIC
void DDController_Init(DDController* pc) {
	memset(pc->phat, 0, DDCTRL_OUTPUTSIZE * sizeof(float));
	memset(pc->phatminusalpha, 0, DDCTRL_OUTPUTSIZE * sizeof(float));
	memset(pc->inputs, 0, DDCTRL_OUTPUTSIZE * sizeof(float));

	arm_mat_init_f32(&pc->Phat,
			DDCTRL_OUTPUTSIZE, 1,
			pc->phat);
	arm_mat_init_f32(&pc->PhatMinusAlpha,
			DDCTRL_OUTPUTSIZE, 1,
			pc->phatminusalpha);
	arm_mat_init_f32(&pc->Inputs,
			DDCTRL_OUTPUTSIZE, 1,
			pc->inputs);
	arm_mat_init_f32(&pc->InvBeta,
			DDCTRL_OUTPUTSIZE,
			DDCTRL_OUTPUTSIZE,
			pc->invbeta);

	pc->initialized = true;
};

void DDController_SetSetpoint(DDController* pc, setpoint_t* ps) {
	memcpy(&pc->ctrl_setpoint, ps, sizeof(setpoint_t));
}

void DDController_SetKxy(DDController* pc, const float k[2]) {
	int i;
	for (i = 0; i < 2; i++) {
		pc->Kxy[i] = k[i];
	}
}

void DDController_SetKz(DDController* pc, const float k[2]) {
	int i;
	for (i = 0; i < 2; i++) {
		pc->Kz[i] = k[i];
	}
}

void DDController_SetKatt(DDController* pc, const float k[2]) {
	int i;
	for (i = 0; i < 2; i++) {
		pc->Katt[i] = k[i];
	}
}

void DDController_SetKyaw(DDController* pc, const float k[2]) {
	int i;
	for (i = 0; i < 2; i++) {
		pc->Kyaw[i] = k[i];
	}
}


void DDController_Step(DDController* pc,
		const state_t *state, DDParams* par,
		float deltaT) {

	if (!pc->initialized) {
		DEBUG_PRINT("DD Controller not initialized!\n");
		DDController_Init(pc);
	}

	// Get the current state estimates
	float x_est[3];
	float y_est[3];
	float z_est[3];
	float roll_est[3];
	float pitch_est[3];
	float yaw_est[3];
	state2arrays(state, x_est, y_est, z_est,
			roll_est, pitch_est, yaw_est);

	// Get the current setpoint
	float x_setpoint[2];
	float y_setpoint[2];
	float z_setpoint[2];
	float yaw_setpoint[2];
	setpoint2arrays(&pc->ctrl_setpoint,
			x_setpoint, y_setpoint, z_setpoint, yaw_setpoint);

	// Compute along Z
	float phatz = pc->Kz[0] * (z_est[0] - z_setpoint[0]) +
		pc->Kz[1] * (z_est[1] - z_setpoint[1]);

	// Compute the Yaw part 
	float phatyaw = pc->Kyaw[0] * (yaw_est[0] - yaw_setpoint[0]) +
		pc->Kyaw[1] * (yaw_est[1] - yaw_setpoint[1]);

	// Compute the mapping position error --> demanded_correction --> demanded_angle
	float phatpitch = DDController_lin2angle(pc,
			x_setpoint, x_est, pitch_est,
			par->alpha_x, par->beta_x, deltaT);

	float phatroll = DDController_lin2angle(pc,
			y_setpoint, y_est, roll_est,
			par->alpha_y, par->beta_y, deltaT);

	pc->phat[0] = phatz;
	pc->phat[1] = phatroll;
	pc->phat[2] = phatpitch;
	pc->phat[3] = phatyaw;

	for (int i = 0; i < 4; i++) {
		phat_log[i] = pc->phat[i];
	}
	
	arm_matrix_instance_f32 Alpha = {
		DDCTRL_OUTPUTSIZE,
		1,
		par->alpha2d};

	arm_matrix_instance_f32 Beta = {
		DDCTRL_OUTPUTSIZE,
		DDCTRL_OUTPUTSIZE,
		par->beta2d};
	
	eval_pseudoinv(&pc->InvBeta, &Beta);

	//arm_mat_sub_f32(&pc->Phat, &Alpha, &pc->PhatMinusAlpha);
	//arm_mat_sub_f32(&pc->Phat, &Alpha, &pc->PhatMinusAlpha);
	//arm_mat_mult_f32(&pc->InvBeta, &pc->PhatMinusAlpha, &pc->Inputs);
	
	// Dirty implementation of the new version (I am reusing Alpha2D variable)
	// Version using the estimated acceleration
	// Phat_minus_Alpha = Desired_acc - Estimated_acc
	par->alpha2d[0] = z_est[2];
	par->alpha2d[1] = roll_est[2];
	par->alpha2d[2] = pitch_est[2];
	par->alpha2d[3] = yaw_est[2];
	arm_mat_sub_f32(&pc->Phat, &Alpha, &pc->PhatMinusAlpha);
	// Computing the DeltaU = InvBeta * Phat_minus_Alpha
	float du[4] = {0,0,0,0};
	arm_matrix_instance_f32 DU = {
		DDCTRL_OUTPUTSIZE,
		1,
		du};
	arm_mat_mult_f32(&pc->InvBeta, &pc->PhatMinusAlpha, &DU);

	// Updating the U
	for (int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
		pc->inputs[i] -= sqrtf(deltaT) * du[i];
	}

	for(int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
		if(pc->inputs[i] > 1.0f){
			pc->inputs[i] = 1.0f;
		} else if(pc->inputs[i] < 0.0f){
			pc->inputs[i] = 0.0f;
		}
	}
}

/**
 * Control Action along a single axis
 */
float DDController_lin2angle(DDController* pc,
		const float setpoint[2],
		const float state_posvel[2],
		const float state_att[2],
		float alpha, float beta,
		float deltaT) {

	float angle_output = 0;

	float pos = state_posvel[0];
	float vel = state_posvel[1];

	float angle = state_att[0];
	float angle_vel = state_att[1];

	// Compute the acc = Kp * (ep) + Kd * (ev)
	float x[2] = {
		pos - setpoint[0],
		vel - setpoint[1]
	};
	float acc_dem = pc->Kxy[0] * x[0] + pc->Kxy[1] * x[1];
	
	// Compute the u to get the acceleration along this axis
	float phix = (-alpha + acc_dem) / beta;
	phix = boundval(phix, -MAXTILT, MAXTILT);

	// That is related to the angle
	float ex[2] = {
		angle - phix,
		angle_vel - 
			(pc->Kxy[0] * vel + pc->Kxy[1] * (alpha + beta * angle)) / beta
	};
	
	angle_output = 
		pc->Katt[0] * ex[0] +
		pc->Katt[1] * ex[1] +
		deltaT * (pc->Kxy[0] * (alpha/beta + angle) + pc->Kxy[1] * angle_vel);

	return angle_output;
}

void DDController_getControls(DDController* pc,
		float m_ctrls[DDCTRL_OUTPUTSIZE]) {
	for (int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
		m_ctrls[i] = pc->inputs[i];
	}
}


LOG_GROUP_START(ctrlDDExtra_log)
	LOG_ADD(LOG_FLOAT, phat_z, &phat_log[0])
	LOG_ADD(LOG_FLOAT, phat_r, &phat_log[1])
	LOG_ADD(LOG_FLOAT, phat_p, &phat_log[2])
	LOG_ADD(LOG_FLOAT, phat_y, &phat_log[3])
LOG_GROUP_STOP(ctrlDDExtra_log)

