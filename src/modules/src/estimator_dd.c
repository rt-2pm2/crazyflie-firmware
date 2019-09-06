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

static bool isInit = false;
static xSemaphoreHandle mutex;

// Timestamps
static uint64_t timestamp;
static uint64_t timestamp_old;
static uint64_t timestamp_ctrl;

float dt_ms;
static uint32_t msg_counter = 0;

// Estimator State 
static double state_z;
static float ctrl_dd;

static uint8_t counter = 0;
static bool updated = false;


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

  isInit = true;
  mutex = xSemaphoreCreateMutex();

	// Initialize the DD Library
	DataDriven_initialize();
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
  timestamp_old = timestamp;

  state_z = pos->z;

 	// Do something with the new measurement 
	// ...	
	// ...	
	DataDriven_U.Measurements = state_z;

	uint64_t tic = usecTimestamp();
	DataDriven_step();
	uint64_t ctrl_exetime = usecTimestamp() - tic;	
	real_T alpha_est = DataDriven_Y.AlphaEstimate;
	real_T beta_est = DataDriven_Y.BetaEstimate;
	float output = (float)DataDriven_Y.Output;
	real_T error = DataDriven_Y.Error;
	real_T minimumT = DataDriven_Y.MinimumT;

  if (msg_counter % 100 == 0) {
		//subcounter++;
		//DEBUG_PRINT("msg_counter = %lu \n", msg_counter);
		DEBUG_PRINT("Execution Time = %llu us\n", ctrl_exetime);
		DEBUG_PRINT("Alpha_est = %.3f\n", (double)alpha_est);
		DEBUG_PRINT("Beta_est = %.3f\n", (double)beta_est);
		DEBUG_PRINT("Error = %.3f\n", (double)error);
		DEBUG_PRINT("Output = %.3f\n", (double)output);
		DEBUG_PRINT("MinimumT = %.3f\n", (double)minimumT);
  }

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
