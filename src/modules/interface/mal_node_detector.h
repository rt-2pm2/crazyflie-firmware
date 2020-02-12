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

// Indexes to access the quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, KC_STATE_DIM
} StateIdx_t;


struct AnchorData {
	distanceMeasurement_t data;
	uint8_t id;
	uint32_t tk_timestamp;
};

struct ActuationData {
	float T;
	float roll;
	float pitch;
}

// The data used by the algorithm.
typedef struct {
  /**
   * The internally-estimated state is:
   * - X, Y, Z: the quad's position in the global frame
   */
  float State[KC_STATE_DIM];


	/*
	 * Anchor Position
	 */
	float APos[NUM_ANCHORS][3];

	/*
	 * Reconstruct
	 */
	float Reconstruct[4][6];

} InternalData_t;


void kalmanCoreInit(kalmanCoreData_t* this);

#endif
