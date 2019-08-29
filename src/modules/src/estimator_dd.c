#include "estimator_dd.h"

#include "FreeRTOS.h"
#include "queue.h"
//#include "task.h"
#include "semphr.h"
#include "debug.h"

#include "sensors.h"

#include "log.h"
#include "param.h"

static bool isInit = false;
static xSemaphoreHandle mutex;

// Timestamps
static uint64_t timestamp;
static uint64_t timestamp_old;
float dt_ms;
static uint32_t msg_counter = 0;

// Estimator State 
static double state_z;

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
	
	return;
}

bool estimatorDDTest(void) {
  return isInit;
}

//static float subcounter = 0;
bool estimatorDDNewMeasurement(const positionMeasurement_t *pos) {

	msg_counter = msg_counter + 1;

	// Measure the timestamp
  timestamp = usecTimestamp(); // Time in microseconds
  dt_ms = (timestamp - timestamp_old)/1000.0;
  timestamp_old = timestamp;

  if (msg_counter % 100 == 0) {
		//subcounter++;
		//DEBUG_PRINT("msg_counter = %lu \n", msg_counter);
		DEBUG_PRINT("dt = %f \n", (double)dt_ms);
  }


  state_z = pos->z;
 	// Do something with the new measurement 
	// ...	
	// ...
	
	if (estimatorReady()) {
		// Update the status of the estimator
		xSemaphoreTake(mutex, portMAX_DELAY);
		updated = true;
		xSemaphoreGive(mutex);
	}

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
