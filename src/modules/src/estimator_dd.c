#include "estimator_dd.h"
#include "DataDriven.h"

#include "FreeRTOS.h"
#include "queue.h"
//#include "task.h"
#include "semphr.h"
#include "debug.h"

#include "stabilizer.h"

#include "sensors.h"

#include "log.h"
#include "param.h"

#include "arm_math.h" 

#define STATE_SIZE (3)
#define BUFF_SIZE (5)
#define TS (0.003)
#define TS2 ((TS) * (TS))

// ===================================
// MEMORY BUFFERS 

// A matrix
float A[STATE_SIZE * STATE_SIZE] = 
{
	1.0,	(TS), (0.5 * TS * TS),
	0.0,	1.0, (TS),
	0.0, 	0.0,	1.0,
};

float O[BUFF_SIZE * STATE_SIZE] = 
{	
	1.0, 	0.0, 		0.0,
	1.0, 	TS, 		TS2/2.0,
	1.0, 	2*TS, 	2.0 * TS2,
	1.0, 	3*TS,		9.0/2.0 * TS2,
	1.0, 	4*TS, 	8.0 * TS2,
};

// C matrix
float C[STATE_SIZE] = {1, 0, 0};

// Pseudo inverse
float O_inv[STATE_SIZE * BUFF_SIZE];


//
// Measurement Buffer
static float Ybuff[BUFF_SIZE];
static float X[STATE_SIZE];


// Temp Buffers for the evaluation of the pseudoinverse
float TempNyNx[BUFF_SIZE * STATE_SIZE];
float TempNxNy[STATE_SIZE * BUFF_SIZE];
float TempNxNx[STATE_SIZE * STATE_SIZE];
float TempNxNx2[STATE_SIZE * STATE_SIZE];




// ====================================	
//
static bool isInit = false;
static xSemaphoreHandle mutex;

// Timestamps
static uint64_t timestamp;
static uint64_t timestamp_old;
static uint64_t timestamp_ctrl;

float dt_ms;
double dt_ms_cum = 0;
static uint32_t msg_counter = 0;

// ====================================
// Estimator State 
static double state_z;
static float ctrl_dd;

static uint8_t counter = 0;
static bool updated = false;

// ====================================
// Filter Data
static int Nmeas = 0;

arm_matrix_instance_f32 Am = {STATE_SIZE, STATE_SIZE, A};
arm_matrix_instance_f32 Cm = {1, STATE_SIZE, C};
arm_matrix_instance_f32 Om = {BUFF_SIZE, STATE_SIZE, O};
arm_matrix_instance_f32 O_invm = {STATE_SIZE, BUFF_SIZE, O_inv};

arm_matrix_instance_f32 TempNyNxm = {BUFF_SIZE, STATE_SIZE, TempNyNx};
arm_matrix_instance_f32 TempNxNym = {STATE_SIZE, BUFF_SIZE, TempNxNy};
arm_matrix_instance_f32 TempNxNxm = {STATE_SIZE, STATE_SIZE, TempNxNx};
arm_matrix_instance_f32 TempNxNx2m = {STATE_SIZE, STATE_SIZE, TempNxNx2};

arm_matrix_instance_f32 Ybuffm = {BUFF_SIZE, 1, Ybuff};
arm_matrix_instance_f32 Xm = {STATE_SIZE, 1, X};

void init_pseudoinv(arm_matrix_instance_f32* Pseudo) {
			arm_mat_trans_f32(&Om, &TempNxNym); // O'
			arm_mat_mult_f32(&TempNxNym, &Om, &TempNxNxm); // (O' x O)
			arm_mat_inverse_f32(&TempNxNxm, &TempNxNx2m); // (O' x O)^-1 x O' = Pseudo inverse
			arm_mat_mult_f32(&TempNxNx2m, &TempNxNym, Pseudo);
}

/** MAYBE IN THE FUTURE
void generate_obs_matrix(arm_matrix_instance_f32* O, int obs_len) {
	int i;

	// Initialize the matrix temp[0] to be A^2
	mat_mult(&Am, &Am, &temp[0]);

	for (i = 2; i < obs_len; i++) {
		mat_mult(&Am, &temp[i%2], &temp[(i + 1)%2]); 
		// The last step put the result in O
		if (i == (obs_len - 1)) 
			mat_mult(&Am, &temp[i%2], O); 	
	}
}
*/

// 
// Circular buffer: not efficient but straightforward...
void update_buffer(float B[BUFF_SIZE], float new) {
	int i;

	if (Nmeas < BUFF_SIZE) {
		Nmeas++;
	}

	for (i = BUFF_SIZE - 1; i > 0; i--) {
		B[i] = B[i - 1];
	}
	B[0] = new;

	return;
}



// ===================================
// Estimator methods

void DDEstimator_step(float y) {
	if (!isInit) {
		estimatorDDInit();
	}

	// Update the buffer
	update_buffer(Ybuff, y);

	// If we have the right amount of 
	// past measurements use the inverse
	// map to estimate the state
	if (Nmeas == BUFF_SIZE) {
		arm_mat_mult_f32(&O_invm, &Ybuffm, &Xm);		
	}
}




// ====================================


// PRIVATE
bool estimatorReady() {
	bool outval = false;
	if (counter >= 10) {
		counter = 0;
		outval = true;
	} else {
		counter++;
	}
	return outval;
}



// PUBLIC
void estimatorDDInit(void) {
  if (isInit)  {
	 	return;
  }

	init_pseudoinv(&O_invm);

  mutex = xSemaphoreCreateMutex();

	// Initialize the DD Library
	DataDriven_initialize();

  isInit = true;
	return;
}

bool estimatorDDTest(void) {
  return isInit;
}

//static float subcounter = 0;
// This function is triggered by the arrival of new measurements
bool estimatorDDNewMeasurement(const positionMeasurement_t *pos) {

	msg_counter = msg_counter + 1;

	// Measure the timestamp
  timestamp = usecTimestamp(); // Time in microseconds
  dt_ms = (timestamp - timestamp_old)/1000.0;
	dt_ms_cum += (double)dt_ms;
  timestamp_old = timestamp;

  state_z = pos->z;

 	// Do something with the new measurement 
	uint64_t tic = usecTimestamp();
	DDEstimator_step(state_z);
	uint64_t ctrl_exetime = usecTimestamp() - tic;	

	float output = 1.0;

if (msg_counter == 1000) {
		//subcounter++;
		DEBUG_PRINT("\n");
		DEBUG_PRINT("Ts (avg) = %f ms\n", dt_ms_cum/1000.0);
		dt_ms_cum = 0.0;
		//DEBUG_PRINT("msg_counter = %lu \n", msg_counter);
		DEBUG_PRINT("Execution Time = %llu us\n", ctrl_exetime);

		DEBUG_PRINT("Y_BUFF	= [");
		for (int i = 0; i < 5; i++) 
			DEBUG_PRINT("%.3f, ", (double)Ybuff[i]);
		DEBUG_PRINT("]\n");

		DEBUG_PRINT("X_est = [%.3f, %.3f, %.3f] \n", (double)X[0], (double)X[1], (double)X[2]);
		msg_counter = 0;
  }



	/*
// Implementation with autogenerated code	 
	DataDriven_U.Measurements = state_z;

	DataDriven_step();
	uint64_t ctrl_exetime = usecTimestamp() - tic;	
	real_T alpha_est = DataDriven_Y.AlphaEstimate;
	real_T beta_est = DataDriven_Y.BetaEstimate;
	float output = (float)DataDriven_Y.Output;
	real_T error = DataDriven_Y.Error;
	real_T minimumT = DataDriven_Y.MinimumT;

  if (msg_counter % 1000 == 0) {
		//subcounter++;
		DEBUG_PRINT("Execution Time = %f ms\n", dt_ms_cum/1000);
		dt_ms_cum = 0.0;
		//DEBUG_PRINT("msg_counter = %lu \n", msg_counter);
		DEBUG_PRINT("Execution Time = %llu us\n", ctrl_exetime);
		DEBUG_PRINT("Alpha_est = %.3f\n", (double)alpha_est);
		DEBUG_PRINT("Beta_est = %.3f\n", (double)beta_est);
		DEBUG_PRINT("Error = %.3f\n", (double)error);
		DEBUG_PRINT("Output = %.3f\n", (double)output);
		DEBUG_PRINT("MinimumT = %.3f\n", (double)minimumT);
		DEBUG_PRINT("Z = %.3f\n", state_z);
		msg_counter = 0;
  }
*/
	if (output != ctrl_dd) {
		ctrl_dd = output;
		dd_controller_push_ctrl(ctrl_dd);
		timestamp_ctrl = usecTimestamp();
	}

//	if (estimatorReady()) {
//		// Update the status of the estimator
//		xSemaphoreTake(mutex, portMAX_DELAY);
//		updated = true;
//		xSemaphoreGive(mutex);
//	}

  return true;
}

double estimatorDDGetEstimatedZ() {
  return state_z;
}

bool estimatorDDHasNewEstimate() {
  bool out;

  xSemaphoreTake(mutex, portMAX_DELAY);
  out = updated;
  if (out)
	updated = false; // Reset the flag
  xSemaphoreGive(mutex);

  return out;
}



// Logging variables
//
LOG_GROUP_START(estimator_dd)
//  LOG_ADD(LOG_FLOAT, est_z, &state_z)
  LOG_ADD(LOG_FLOAT, sens_dt_ms, &dt_ms)
//  LOG_ADD(LOG_FLOAT, msg_cnt, &subcounter)
LOG_GROUP_STOP(estimator_dd)
