/**
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
 *
 */

#ifndef __MAL_NODE_DETECTOR_H__
#define __MAL_NODE_DETECTOR_H__

#include "cf_math.h"
#include "stabilizer_types.h"

#define NUM_ANCHORS (8)


typedef struct {
	distanceMeasurement_t data;
	uint8_t id;
	uint32_t tk_timestamp;
} anchor_data_t;

typedef struct {
	float T;
	float roll;
	float pitch;
} actuation_data_t;

struct AlgorithmData {
	actuation_data_t act_d;
	anchor_data_t anch_d[NUM_ANCHORS];
	uint32_t tk_timestamp;
};

// The data used by the algorithm.
typedef struct {
	/**
	 * The internally-estimated state is:
	 * - X, Y, Z: the quad's position in the global frame
	 */

	/*
	 * Anchor Position
	 */
	float APos[NUM_ANCHORS][3];

	/*
	 * Reconstruct
	 */
	float Reconstruct[4][6];

	/*
	 * Averages
	 */
	Axis3f accAverage;

	float thrustAverage;

	float rollAverage;
	float pitchAverage;

} InternalData_t;

/**
 * Initialization Function
 */
void MND_Init();


void MND_update_dyn(state_t* state,
		Axis3f* acc,
		float thrust,
		const uint32_t tick);

void MND_update_meas(
		distanceMeasurement_t* dist,
		uint8_t anchor_index,
		const uint32_t tick);

#endif
