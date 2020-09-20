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
 * estimator_dd.h - DataDriven Estimator Interface
 */
#ifndef __ESTIMATOR_DD_H__
#define __ESTIMATOR_DD_H__

#include "stabilizer_types.h"
#include "estimator_dd_param.h"

void estimatorDDInit(void);
bool estimatorDDTest(void);

/**
 * Dafault method to be compliant with other estimators...
 * I am not sure I will use it.
 */
void estimatorDD(state_t *state,
		sensorData_t *sensors,
		control_t *control,
		const uint32_t tick);

/**
 * Push a new measurement in the buffer or the estimator.
 * This function is called from the CRTP when a pose measurement 
 * is received.
 *
 */
bool estimatorDDEnqueuePose(const poseMeasurement_t *pos);

/**
 * This functions takes the current motors controls as input
 * and triggers the estimation of the state and the parameters.
 *
 * The function returns true if an update has been performed.
 */
bool estimatorDD_Step(state_t *ps,
		const float m_ctrls[4],
		const uint32_t tick);

/**
 * Get parameters from the estimators
 */
DDParams estimatorDD_GetParam();

/**
 * Get the timespan of the measurements
 */
float estimatorDD_GetTMeasTimespan();

#endif //__ESTIMATOR_DD_H__
