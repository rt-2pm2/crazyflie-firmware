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
	1.0,	-(TS), (0.5 * TS * TS),
	0.0,	1.0, -(TS),
	0.0, 	0.0,	1.0,
};

float O[BUFF_SIZE * STATE_SIZE] = 
{	
	1.0, 	0.0, 	0.0,
	1.0, 	-TS, 	TS2/2,
	1.0, 	-2*TS, 	2*TS2,
	1.0, 	-3*TS,	9/2*TS2,
	1.0, 	-4*TS, 	8*TS2,
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
//static uint64_t timestamp_ctrl;

float dt_ms;
double dt_ms_cum = 0;
static uint32_t msg_counter = 0;

// ====================================
// Estimator State 
static double state_z;
//static float ctrl_dd;

static uint8_t counter = 0;
static bool updated = false;

// ====================================
// Filter Data
static int Nmeas;

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


// ===================================
// Estimator methods

void DDEstimator_step(float y) {
	if (!isInit) {
		estimatorDDInit();
	}

	// Update the buffer
	Ybuff[Nmeas % BUFF_SIZE] = y;
	Nmeas++;

	if (Nmeas == BUFF_SIZE) {
		Nmeas = 0;
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

// This function is triggered by the arrival of new measurements
bool estimatorDDNewMeasurement(const positionMeasurement_t *pos) {

	msg_counter = msg_counter + 1;

	// Measure the timestamp
	timestamp = usecTimestamp(); // Time in microseconds
	dt_ms = (timestamp - timestamp_old)/1000.0;
	timestamp_old = timestamp;

	state_z = pos->z;

	// Do something with the new measurement 
	DDEstimator_step(state_z);

//	if (msg_counter == 1000) {
//		DEBUG_PRINT("\n");
//		DEBUG_PRINT("Execution Time = %llu us\n", ctrl_exetime);
//
//		DEBUG_PRINT("Y_BUFF	= [");
//		for (int i = 0; i < 5; i++) 
//			DEBUG_PRINT("%.3f, ", (double)Ybuff[i]);
//		DEBUG_PRINT("]\n");
//
//		DEBUG_PRINT("X_est = [%.3f, %.3f, %.3f] \n", (double)X[0], (double)X[1], (double)X[2]);
//		msg_counter = 0;
//	}


//	if (output != ctrl_dd) {
//		ctrl_dd = output;
//		dd_controller_push_ctrl(ctrl_dd);
//		timestamp_ctrl = usecTimestamp();
//	}

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
	LOG_ADD(LOG_FLOAT, est_x, &X[0])
	LOG_ADD(LOG_FLOAT, est_xd, &X[1])
	LOG_ADD(LOG_FLOAT, est_xdd, &X[2])
	LOG_ADD(LOG_FLOAT, sens_dt_ms, &dt_ms)
LOG_GROUP_STOP(estimator_dd)
