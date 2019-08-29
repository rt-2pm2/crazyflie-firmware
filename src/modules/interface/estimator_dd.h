#ifndef __ESTIMATOR_DD_H__
#define __ESTIMATOR_DD_H__

#include <stdint.h>
#include "stabilizer_types.h"

void estimatorDDInit(void);

bool estimatorDDTest(void);

// Not used...
void estimatorDD(state_t* state, sensorData_t* sensors, control_t* control, const uint32_t tick);

// Push the new sensor reading in the estimator
bool estimatorDDNewMeasurement(const positionMeasurement_t *pos);

// Get the estimated value
double estimatorDDGetEstimatedZ();

// Check whether the estimator has a finished the estimation cycle
bool estimatorDDHasNewEstimate();


#endif
