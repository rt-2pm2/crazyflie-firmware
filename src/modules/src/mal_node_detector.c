/**
 * Authored by  ,  2020
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
 * =====================================================================
 *
 */

#include "mal_node_detector.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#include "estimator.h"

#define DEBUG_MODULE "MNDETECTOR"

#include "debug.h"

#define Xind (0)
#define Yind (1)
#define DEG2RAD (3.14f / 180.0f)

#define NUM_MAX_MALICIOUS (6)

// Semaphore to signal that we got data from the stabilzer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


// Vehicle Mass
#define CF_MASS (0.027f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/(CF_MASS * 1000) /65536.0f)


/**
 * Task period in [ms]
 */
#define TASKPERIOD (30) 

/*
 * Variables Identifying the 2 good anchors
 */
static uint8_t base0 = 6;
static uint8_t base1 = 7;

static bool firstTime = true;

static float curr_z = 0.0;

static float detection_threshold = 0.1;
static bool attack_detected = false;

/**
 * Algorithm Data Structure
 */
static InternalData_t MND_Data;


/**
 * Vector of distance measurements 
 * Each element is a structure: 
 *
 *	typedef struct distanceMeasurement_s {
 *		union {
 *			struct {
 *				float x;
 *				float y;
 *				float z;
 *			};
 *			float pos[3];
 *		};
 *		float distance;
 *		float stdDev;
 *	} distanceMeasurement_t;
 *
 */
static anchor_data_t AnchorMeas[NUM_ANCHORS]; 
static anchor_data_t AnchorMeasBuffer[2][NUM_ANCHORS] ;

/*
 * Actuation data and respective buffer
 * The actuation is considere the Forces along x,y and z
 */
static float Actuation[3];
static float ActuationBuffer[2][3];

/**
 * Internal variables.
 */
static bool isInit = false;

static Axis3f accAccumulator;
static uint32_t accAccumulatorCount;

static float thrustAccumulator;
static uint32_t thrustAccumulatorCount;

static float rollAccumulator;
static uint32_t rollAccumulatorCount;

static float pitchAccumulator;
static uint32_t pitchAccumulatorCount;

static uint32_t last_activation = 0;
static uint8_t activated_mnd = false;

static int8_t outcome[NUM_MAX_MALICIOUS];

// Index for the buffers
static uint8_t curr_row = 0;

// Residuals for the detection
static float residual[NUM_MAX_MALICIOUS];


// ===========================================================
// ===========================================================


// Internal Functions
static void MND_Task(void* parameters);
STATIC_MEM_TASK_ALLOC(MND_Task, 2 * configMINIMAL_STACK_SIZE);

static void initAnchorsPosition();
static bool averaging();
static int invertColumnMajor(float n[4][4], float invOut[4][4]);
static void build_Q(float Q[4][4], int i, float delta);
static void MatrixMultiVec( const float m[4][4], const float vec[4][1], float output[4][1]);
static void preprocessing(int num, const anchor_data_t AnchorData[NUM_ANCHORS], float prep_meas[NUM_ANCHORS], float z_drone);
static void prep_actuation(float T, float r, float p, float Fxyz[3]);
static bool rule(float residual[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]);


// ===========================================================



static void MND_Task(void* parameters) {
	systemWaitStart();

	uint32_t current_tick = xTaskGetTickCount();
	uint32_t previous_tick = xTaskGetTickCount();

	while (true) {
		xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

		// When the task is waken up it will save the 
		// current status of the anchors and actuation.

		// If there are at least two updates it can
		// run the algorithm for detecting malicious nodes

		xSemaphoreTake(dataMutex, portMAX_DELAY);	
		if (firstTime) {
			curr_row = 0;
			firstTime = false;
		} else {
			curr_row = (curr_row + 1) % 2;
		}

		memcpy(AnchorMeasBuffer[curr_row], AnchorMeas, sizeof(AnchorMeas));	
		memcpy(ActuationBuffer[curr_row], Actuation, 3 * sizeof(float));

		if (!firstTime) {
			// Allocate variables
			float prep_meas1[NUM_ANCHORS];
			float prep_meas2[NUM_ANCHORS];
			float prep_meas3[NUM_ANCHORS];
			float prep_meas4[NUM_ANCHORS];

			current_tick  = xTaskGetTickCount();
			float delta = current_tick - previous_tick;

			// Processing the old measurement using the two bases
			preprocessing(base0, AnchorMeasBuffer[(curr_row + 1) % 2], prep_meas1, curr_z); 
			preprocessing(base1, AnchorMeasBuffer[(curr_row + 1) % 2], prep_meas2, curr_z);

			// Processing the current measurement using the two bases
			preprocessing(base0, AnchorMeasBuffer[curr_row], prep_meas3, curr_z);
			preprocessing(base1, AnchorMeasBuffer[curr_row], prep_meas4, curr_z);

			float recons[4][NUM_MAX_MALICIOUS];  // used to store the reconstruct of each beacon

			for (int i = 0; i<NUM_MAX_MALICIOUS; i++) {
				float Y[4][1];
				float Q[4][4];
				float temp[4][1];

				build_Q(Q,i, delta);

				Y[0][0] = prep_meas1[i];
				Y[1][0] = prep_meas2[i];
				Y[2][0] = prep_meas3[i] - 
					(MND_Data.APos[i][0] - MND_Data.APos[base0][0])*ActuationBuffer[(curr_row + 1) % 2][Xind]/CF_MASS -
					(MND_Data.APos[i][1] - MND_Data.APos[base0][1])*ActuationBuffer[(curr_row + 1) % 2][Yind]/CF_MASS;
				Y[3][0] = prep_meas4[i] -
					(MND_Data.APos[i][0] - MND_Data.APos[base1][0])*ActuationBuffer[(curr_row + 1) % 2][Xind]/CF_MASS -
					(MND_Data.APos[i][1] - MND_Data.APos[base1][1])*ActuationBuffer[(curr_row + 1) % 2][Yind]/CF_MASS;

				MatrixMultiVec(Q,Y,temp);

				for (int j=0; j<4; j++) {
					recons[j][i] = temp[j][0];
				}
			}

			float W[4] = {100,1,100,1}; //CF_MASS matrix

			for (int j = 0; j < NUM_MAX_MALICIOUS; j++) {
				residual[j] = W[0]*recons[0][j] +
					W[1]*recons[1][j] +
					W[2]*recons[2][j] +
					W[3]*recons[3][j];
			}

			attack_detected = rule(residual, outcome);

			//DEBUG_PRINT("%u | %u | %u \n", outcome[0], outcome[1], outcome[2]);
		}	

		xSemaphoreGive(dataMutex);

	}
}


/**
 * Update the value of the anchor measurements
 */
void MND_update_meas(
		distanceMeasurement_t* dist,
		uint8_t anchor_index,
		const uint32_t tick) {

	//Lock the data mutex
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Update anchors data
	AnchorMeas[anchor_index].data = *dist;
	AnchorMeas[anchor_index].tk_timestamp = tick;


	// It's redundant, but I will clean it up later...
	if (attack_detected && activated_mnd) {
		// If an attack has been detected check before
		// sending the distance information to the filter
		if (anchor_index != outcome[0]) {
			estimatorEnqueueDistance(dist);
		}
	} else {
		estimatorEnqueueDistance(dist);
	}
			
	// Release the data mutex
	xSemaphoreGive(dataMutex);


}

/*
 * Feed new model data into the Malicious Node Detector
 */
void MND_update_dyn(
		state_t* state,
		Axis3f* acc,
		float thrust,
		const uint32_t tick) {

	//Lock the data mutex
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Average Acceleration.
	accAccumulator.x += acc->x;
	accAccumulator.y += acc->y;
	accAccumulator.z += acc->z;
	accAccumulatorCount++;

	// Averate Thrust
	thrustAccumulator += thrust;
	thrustAccumulatorCount++;

	// Average Roll and Pitch
	// (I change the sign to the pitch to counteract the 
	// weird Bitcraze convention)
	rollAccumulator += state->attitude.roll;
	rollAccumulatorCount++;
	pitchAccumulator += -state->attitude.pitch;
	pitchAccumulatorCount++;

	curr_z = state->position.z;


	// Unlock the data mutex
	xSemaphoreGive(dataMutex);

	// If a given amount of time is passed
	// 	Unlock the task
	if (tick - last_activation > TASKPERIOD) {
		if (averaging()) { 
			last_activation = tick;
			xSemaphoreGive(runTaskSemaphore);
		}
	}
}

/**
 * Average the last received data
 */
static bool averaging() {
	if (accAccumulatorCount == 0 || thrustAccumulatorCount == 0)
	{
		return false;
	}

	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Gs -->  ms^-2
	MND_Data.accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
	MND_Data.accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
	MND_Data.accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

	// Grams --> ms^-2
	MND_Data.thrustAverage = thrustAccumulator * CONTROL_TO_ACC / thrustAccumulatorCount;

	MND_Data.rollAverage = rollAccumulator * DEG2RAD / rollAccumulatorCount;
	MND_Data.pitchAverage = pitchAccumulator * DEG2RAD / pitchAccumulatorCount;

	// Convert the Actuation data into a F_xyz
	// The pitch should be inverted because of the weird bitcraze convention.
	prep_actuation(MND_Data.thrustAverage,
			MND_Data.rollAverage,
			MND_Data.pitchAverage, 
			Actuation);

	// Reset the averaging variables
	accAccumulator = (Axis3f){.axis={0}};
	accAccumulatorCount = 0;
	thrustAccumulator = 0;
	thrustAccumulatorCount = 0;

	rollAccumulator = 0;
	rollAccumulatorCount = 0;

	pitchAccumulator = 0;
	pitchAccumulatorCount = 0;


	xSemaphoreGive(dataMutex);

	return true;
}



void prep_actuation(float T, float r, float p, 
		float Fxyz[3]) {

	// Scaling and drift removing
	float T_n = T; //  * scale; // Force in Newton
	float p_ = p; // + drift1;
	float r_ = r; // + drift2;  // correct scale and drift

	Fxyz[2] = T_n;
	// Computing Fx and Fy
	Fxyz[0] = T_n * sinf(p_);
	Fxyz[1] = -T_n * cosf(p_) * sinf(r_);
}


/**
 * Preprocessing of the input
 */
void preprocessing(int num, const anchor_data_t AnchorData[NUM_ANCHORS], float prep_meas[NUM_ANCHORS], float z_drone) {
	// Temp variables
	float meas2[NUM_ANCHORS];
	float meas2_d[NUM_ANCHORS];

	// Square the distances from the anchors and put the 
	// result in the temporary vector meas2
	for (int i = 0; i < NUM_ANCHORS; i++) {
		meas2[i] = powf(AnchorData[i].data.distance, 2);
	}

	// Select the desired anchor
	float anchor = meas2[num];

	for (int i = 0; i < NUM_ANCHORS; i++) 
		meas2_d[i] = anchor - meas2[i];

	for (int i = 0; i < NUM_ANCHORS; i++) {
		meas2_d[i] = meas2_d[i] + 
			powf(MND_Data.APos[i][0],2) + 
			powf(MND_Data.APos[i][1],2) + 
			powf(MND_Data.APos[i][2],2);

		meas2_d[i] = meas2_d[i] - 
			powf(MND_Data.APos[num][0],2) - 
			powf(MND_Data.APos[num][1],2) - 
			powf(MND_Data.APos[num][2],2);

		meas2_d[i] = meas2_d[i] - 
			2*(MND_Data.APos[i][2] - MND_Data.APos[num][2])*z_drone;

		// Update the output data
		prep_meas[i] = meas2_d[i];
	}

}  

void build_Q(float Q[4][4], int i, float delta) {

	float O[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	O[0][0] = 2*(MND_Data.APos[i][0]-MND_Data.APos[base0][0]);
	O[0][2] = 2*(MND_Data.APos[i][1]-MND_Data.APos[base0][1]);
	O[1][0] = 2*(MND_Data.APos[i][0]-MND_Data.APos[base1][0]);
	O[1][2] = 2*(MND_Data.APos[i][1]-MND_Data.APos[base1][1]);
	O[2][0] = O[0][0];
	O[2][2] = O[0][2]; 
	O[3][0] = O[1][0];
	O[3][2] = O[1][2];
	O[2][1] = O[2][0]*delta;
	O[2][3] = O[2][2]*delta;
	O[3][1] = O[3][0]*delta;
	O[3][3] = O[3][2]*delta;

	invertColumnMajor(O,Q);
}


// This computes a 4*4 matrix multiply a 4*1 vector
void MatrixMultiVec( const float m[4][4], const float vec[4][1], float output[4][1]) {
	output[0][0] = m[0][0]*vec[0][0] + m[0][1]*vec[1][0] + m[0][2]*vec[2][0] + m[0][3]*vec[3][0];
	output[1][0] = m[1][0]*vec[0][0] + m[1][1]*vec[1][0] + m[1][2]*vec[2][0] + m[1][3]*vec[3][0];
	output[2][0] = m[2][0]*vec[0][0] + m[2][1]*vec[1][0] + m[2][2]*vec[2][0] + m[2][3]*vec[3][0];
	output[3][0] = m[3][0]*vec[0][0] + m[3][1]*vec[1][0] + m[3][2]*vec[2][0] + m[3][3]*vec[3][0];
}

// This is the invert of a 4*4 matrix
int invertColumnMajor(float n[4][4], float invOut[4][4]) {
	float m[16] = {n[0][0],n[0][1],n[0][2],n[0][3],n[1][0],n[1][1],n[1][2],n[1][3],n[2][0],n[2][1],n[2][2],n[2][3],n[3][0],n[3][1],n[3][2],n[3][3]};

	float inv[16], det;
	int i,j;

	inv[ 0] =  m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[ 4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[ 8] =  m[4] * m[ 9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[ 9];
	inv[12] = -m[4] * m[ 9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[ 9];
	inv[ 1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[ 5] =  m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[ 9] = -m[0] * m[ 9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[ 9];
	inv[13] =  m[0] * m[ 9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[ 9];
	inv[ 2] =  m[1] * m[ 6] * m[15] - m[1] * m[ 7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[ 7] - m[13] * m[3] * m[ 6];
	inv[ 6] = -m[0] * m[ 6] * m[15] + m[0] * m[ 7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[ 7] + m[12] * m[3] * m[ 6];
	inv[10] =  m[0] * m[ 5] * m[15] - m[0] * m[ 7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[ 7] - m[12] * m[3] * m[ 5];
	inv[14] = -m[0] * m[ 5] * m[14] + m[0] * m[ 6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[ 6] + m[12] * m[2] * m[ 5];
	inv[ 3] = -m[1] * m[ 6] * m[11] + m[1] * m[ 7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[ 9] * m[2] * m[ 7] + m[ 9] * m[3] * m[ 6];
	inv[ 7] =  m[0] * m[ 6] * m[11] - m[0] * m[ 7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[ 8] * m[2] * m[ 7] - m[ 8] * m[3] * m[ 6];
	inv[11] = -m[0] * m[ 5] * m[11] + m[0] * m[ 7] * m[ 9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[ 9] - m[ 8] * m[1] * m[ 7] + m[ 8] * m[3] * m[ 5];
	inv[15] =  m[0] * m[ 5] * m[10] - m[0] * m[ 6] * m[ 9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[ 9] + m[ 8] * m[1] * m[ 6] - m[ 8] * m[2] * m[ 5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if(det == 0)
		return 0;

	det = 1.f / det;

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			invOut[i][j] = inv[4*i+j] * det;
		}
	}
	return 1;
}

//find out the guy that deviates the average most
bool rule(float residual[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]) {
	float sum = 0;
	bool ordered = true;
	float diff_residual;
	bool detected = false;

	for (int i = 0; i < NUM_MAX_MALICIOUS; i++) {
		sum += residual[i];
	}

	// Take the average of the comulative residual
	sum = sum / NUM_MAX_MALICIOUS;

	for (int i=0; i < NUM_MAX_MALICIOUS; i++) {
		residual[i] -= sum;
		residual[i] = fabsf(residual[i]);
	}

	for (int i = 0; i < NUM_MAX_MALICIOUS; i++) {
			outcome[i] = i;
	}

	while (!ordered) {
		ordered = true;
		for (int i = 0; i < 5; i++) {
			if (residual[i] < residual[i+1]) {
				float a = residual[i];
				float b = outcome[i];

				residual[i] = residual [i+1];
				outcome[i] = outcome[i+1];

				residual[i+1] = a;
				outcome[i+1] = b;

				ordered = false;
			}
		}
	}
	
	// Compute the difference between the first 2  residuals to understand 
	// if there was actually an attack.
	diff_residual = fabs(residual[0] - residual[1]);
	if (diff_residual < detection_threshold) {
		int i = 0;

		// Reset the array to a definite value to indicate 
		// that there was no malicious attack.
		for (i = 0; i < NUM_MAX_MALICIOUS; i++) {
			outcome[i] = -1;
		}
		detected = false;
	} else {
		detected = true;
	}

	return detected; 
}


static void initAnchorsPosition() {
	MND_Data.APos[0][0] = 2.60;
	MND_Data.APos[0][1] = -2.05;
	MND_Data.APos[0][2] = 0.0;

	MND_Data.APos[1][0] = -2.66;
	MND_Data.APos[1][1] = -2.05;
	MND_Data.APos[1][2] = 0.0;

	MND_Data.APos[2][0] = -2.66;
	MND_Data.APos[2][1] = 1.47;
	MND_Data.APos[2][2] = 0.0;

	MND_Data.APos[3][0] = 2.60;
	MND_Data.APos[3][1] = 1.47;
	MND_Data.APos[3][2] = 0.74;

	MND_Data.APos[4][0] = -0.12;
	MND_Data.APos[4][1] = -2.10;
	MND_Data.APos[4][2] = 1.90;

	MND_Data.APos[5][0] = 1.62;
	MND_Data.APos[5][1] = -2.76;
	MND_Data.APos[5][2] = 1.40;

	MND_Data.APos[6][0] = -0.99;
	MND_Data.APos[6][1] = 1.45;
	MND_Data.APos[6][2] = 0.76;

	MND_Data.APos[7][0] = -2.39;
	MND_Data.APos[7][1] = 0;
	MND_Data.APos[7][2] = 0.78;
}

void MND_Init() {
	int i;

	// Initialize the semaphores
	vSemaphoreCreateBinary(runTaskSemaphore);
	dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

	// Initialize the anchors positions
	initAnchorsPosition();

	// Initialize the variables related to 
	// actuation.
	accAccumulator = (Axis3f){.axis={0}};
	accAccumulatorCount = 0;

	thrustAccumulator = 0;
	thrustAccumulatorCount = 0;

	rollAccumulator = 0;
	rollAccumulatorCount = 0;

	pitchAccumulator = 0;
	pitchAccumulatorCount = 0;

	isInit = true;

	// Initialize the value of the outcomes
	for (i = 0; i < NUM_MAX_MALICIOUS; i++) {
		outcome[i] = -1;
	}
	
	STATIC_MEM_TASK_CREATE(
			MND_Task,
			MND_Task, MND_TASK_NAME,
			NULL, MND_TASK_PRI);
}


// Temporary development groups
LOG_GROUP_START(mnd_log)
	LOG_ADD(LOG_INT8, outcome0, &outcome[0])
	LOG_ADD(LOG_FLOAT, residual0, &residual[0])
	LOG_ADD(LOG_FLOAT, residual1, &residual[1])
LOG_GROUP_STOP(mnd_log)

LOG_GROUP_START(mnd_dgb_log)
	LOG_ADD(LOG_FLOAT, thrustAvg, &MND_Data.thrustAverage)
	LOG_ADD(LOG_FLOAT, rollAvg, &MND_Data.rollAverage)
	LOG_ADD(LOG_FLOAT, pitchAvg, &MND_Data.pitchAverage)
LOG_GROUP_STOP(mnd_dbg_log)

PARAM_GROUP_START(mnd_param)
	PARAM_ADD(PARAM_UINT8, activate, &activated_mnd)
	PARAM_ADD(PARAM_UINT8, base0, &base0)
	PARAM_ADD(PARAM_UINT8, base1, &base1)
	PARAM_ADD(PARAM_FLOAT, threshold, &detection_threshold)
PARAM_GROUP_STOP(mnd_param)
