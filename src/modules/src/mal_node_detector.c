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

#include "statsCnt.h"

#define DEBUG_MODULE "MNDETECTOR"
#include "debug.h"

#define Xind (0)
#define Yind (1)

// Distance-to-point measurements
static xQueueHandle distDataQueue;
STATIC_MEM_QUEUE_ALLOC(distDataQueue, 10, sizeof(distanceMeasurement_t));


// Semaphore to signal that we got data from the stabilzer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


/**
 * Constants used in the estimator
 */

#define CF_MASS (0.027f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/(CF_MASS * 1000) /65536.0f)


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz



/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z:  position of the vehicle
 *
 * For more information, refer to the paper
 */
static MND_Data_t MND_Data;

/**
 * Vector of distance measurements 
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
static AnchorData[8] AnchorMeas; 

/**
 * Internal variables.
 */

static bool isInit = false;

static Axis3f accAccumulator;
static float thrustAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;

static bool quadIsFlying = false;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);

/**
 * Supporting and utility functions
 */

static void MND_Task(void* parameters);
static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick);

STATIC_MEM_TASK_ALLOC(MND_Task, 2 * configMINIMAL_STACK_SIZE);

// --------------------------------------------------


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

// Called one time during system startup
void MND_DetectionTaskInit() {
	distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);

	vSemaphoreCreateBinary(runTaskSemaphore);
	dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

	STATIC_MEM_TASK_CREATE(MND_Task, MND_Task, MND_TASK_NAME, NULL, MND_TASK_PRI);

	isInit = true;
}

bool MND_TaskTest() {
	return isInit;
}

static void MND_Task(void* parameters) {
	systemWaitStart();

	uint32_t lastPrediction = xTaskGetTickCount();
	uint32_t nextPrediction = xTaskGetTickCount();

	while (true) {
		xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

		float prep_meas1[11];
		float prep_meas2[11];
		float prep_meas3[11];
		float prep_meas4[11];

		float Thr = actuation.thrust;
		float roll = actuation.roll;
		float pitch = actuation.pitch;

		float F[3];

		uint32_t osTick = xTaskGetTickCount();

		preprocessing_actuation(Thr, roll, pitch, F[3]);
		
		preprocessing(6, AnchorMeas, prep_meas1, z_drone); 
		preprocessing(7, AnchorMeas, prep_meas2, z_drone);
		preprocessing(6, AnchorMeas, prep_meas3, z_drone);
		preprocessing(7, AnchorMeas, prep_meas4, z_drone);

		float recons[4][6];  // used to store the reconstruct of each beacon

		for (int i = 0; i<6; i++)
		{
			float O[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

			O[0][0] = 2*(MND_Data.APos[i][0]-MND_Data.APos[6][0]);
			O[0][2] = 2*(MND_Data.APos[i][1]-MND_Data.APos[6][1]);
			O[1][0] = 2*(MND_Data.APos[i][0]-MND_Data.APos[7][0]);
			O[1][2] = 2*(MND_Data.APos[i][1]-MND_Data.APos[7][1]);
			O[2][0] = O[0][0];
			O[2][2] = O[0][2]; 
			O[3][0] = O[1][0];
			O[3][2] = O[1][2];
			O[2][1] = O[2][0]*delta;
			O[2][3] = O[2][2]*delta;
			O[3][1] = O[3][0]*delta;
			O[3][3] = O[3][2]*delta;

			float Q[4][4];
			invertColumnMajor(O,Q);

			float Y[4][1];
			Y[0][0] = prep_meas1[i];
			Y[1][0] = prep_meas2[i];
			Y[2][0] = prep_meas3[i] - 
				(MND_Data.APos[i][0] - MND_Data.APos[6][0])*F1[Xind]/CF_MASS -
				(MND_Data.APos[i][1] - MND_Data.APos[6][1])*F1[Yind]/CF_MASS;

			Y[3][0] = prep_meas4[i] -
				(MND_Data.APos[i][0] - MND_Data.APos[7][0])*F2[Xind]/CF_MASS -
				(MND_Data.APos[i][1] - MND_Data.APos[7][1])*F2[Yind]/CF_MASS;

			float temp[4][1];
			MatrixMultiVec(Q,Y,temp);

			for (int j=0; j<4; j++) {
				recons[j][i] = temp[j][1];
			}
		}

		float W[4] = {100,1,100,1}; //CF_MASS matrix
		float voting[6];
		int outcome[6];

		for (int j = 0; j < 6; j++) {
			voting[j] = W[0]*recons[0][j] + W[1]*recons[1][j] + W[2]*recons[2][j] + W[3]*recons[3][j];
		}

		rule(voting, outcome);
			nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);
	
		xSemaphoreTake(dataMutex, portMAX_DELAY);
		xSemaphoreGive(dataMutex);

		STATS_CNT_RATE_EVENT(&updateCounter);
	}
}


/**
 * Update the value of the anchor measurements
 */
void mn_detector_update_meas(
		distanceMeasurement_t* dist,
		uint8_t anchor_index,
		const uint32_t tick) {

	//Lock the data mutex
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Update anchors data
	AnchorMeas[anchor_index].data = *dist;
	AnchorMeas[anchor_index].tk_timestamp = tick;

	// Release the data mutex
	xSemaphoreGive(dataMutex);

	// Unlock the task
	xSemaphoreGive(runTaskSemaphore);
}

/*
 * Feed new model data into the Malicious Node Detector
 */
void mn_detector_update_dyn(
		state_t *state,
		sensorData_t *sensors,
		control_t *control,
		const uint32_t tick) {

	//Lock the data mutex
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Average the last IMU measurements.
	if (sensorsReadAcc(&sensors->acc)) {
		accAccumulator.x += sensors->acc.x;
		accAccumulator.y += sensors->acc.y;
		accAccumulator.z += sensors->acc.z;
		accAccumulatorCount++;
	}

	// Make a copy of sensor data to be used by the task
	memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));

	// Copy the latest state, calculated by the task
	memcpy(state, &taskEstimatorState, sizeof(state_t));

	// Unlock the data mutex
	xSemaphoreGive(dataMutex);
}

static bool predictStateForward(uint32_t osTick, float dt) {
	if (accAccumulatorCount == 0 || thrustAccumulatorCount == 0)
	{
		return false;
	}

	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// accelerometer is in Gs but the estimator requires ms^-2
	Axis3f accAverage;
	accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
	accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
	accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

	// thrust is in grams, we need ms^-2
	float thrustAverage = thrustAccumulator * CONTROL_TO_ACC / thrustAccumulatorCount;

	accAccumulator = (Axis3f){.axis={0}};
	accAccumulatorCount = 0;
	thrustAccumulator = 0;
	thrustAccumulatorCount = 0;

	xSemaphoreGive(dataMutex);

	return true;
}


static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick) {
	bool doneUpdate = false;

	distanceMeasurement_t dist;
	while (stateEstimatorHasDistanceMeasurement(&dist))
	{
		kalmanCoreUpdateWithDistance(&coreData, &dist);
		doneUpdate = true;
	}

	return doneUpdate;
}

// Called when this estimator is activated
void MNDInit(void) {
	xQueueReset(distDataQueue);

	xSemaphoreTake(dataMutex, portMAX_DELAY);
	accAccumulator = (Axis3f){.axis={0}};
	thrustAccumulator = 0;

	accAccumulatorCount = 0;
	thrustAccumulatorCount = 0;
	xSemaphoreGive(dataMutex);

	kalmanCoreInit(&coreData);
}

static bool appendMeasurement(xQueueHandle queue, void *measurement)
{
	portBASE_TYPE result;
	bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

	if (isInInterrupt) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE)
		{
			portYIELD();
		}
	} else {
		result = xQueueSend(queue, measurement, 0);
	}

	if (result == pdTRUE) {
		STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
		return true;
	} else {
		STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
		return true;
	}
}


bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
	ASSERT(isInit);
	return appendMeasurement(distDataQueue, (void *)dist);
}


bool estimatorKalmanTest(void)
{
	return isInit;
}


void preprocessing_actuation(float T, float r, float p, 
		float Fxyz[3]) {

	// Scaling and drift removing
	float T_n = T * scale; // Force in Newton
	float p_ = p + drift1;
	float r_ = r + drift2;  // correct scale and drift

	// Convert to radiant
	r_ = r_ * pi / 180;
	p_ = p_ * pi / 180;

	Fxyz[2] = T_n;
	// Computing Fx and Fy
	Fxyz[0] = -T_n * sin(p_);
	Fxyz[1] = -T_n * cos(p_) * sin(r_);
}


/**
 * Preprocessing of the input
 */
void preprocessing(int num, const AnchorData[NUM_ANCHORS], float prep_meas[NUM_ANCHORS], float z_drone){
	// Temp variables
	float meas2[NUM_ANCHORS];
	float meas2_d[NUM_ANCHORS];

	// Square the distances from the anchors and put the 
	// result in the temporary vector meas2
	for (int i = 0; i < NUM_ANCHORS; i++) {
		meas2[i] = pow(AnchorData[i].data.dist, 2);
	}

	// Select the desired anchor
	float anchor = meas2[num];

	for (int i = 0; i < NUM_ANCHORS; i++) 
		meas2_d[i] = anchor - meas2[i];

	for (int i = 0; i < NUM_ANCHORS; i++) {
		meas2_d[i] = meas2_d[i] +
			pow(MND_Data.APos[i][0],2) + 
			pow(MND_Data.APos[i][1],2) +
			pow(MND_Data.APos[i][2],2);

		meas2_d[i] = meas2_d[i] -
			pow(MND_Data.APos[num][0],2) - 
			pow(MND_Data.APos[num][1],2) - 
			pow(MND_Data.APos[num][2],2);

		meas2_d[i] = meas2_d[i] - 
			2*(MND_Data.APos[i][2] - MND_Data.APos[num][2])*z_drone;

		// Update the output data
		prep_meas[i] = meas2_d[i];
	}

}  


// This computes a 4*4 matrix multiply a 4*1 vector
void MatrixMultiVec(
		const float m[4][4],
		const float vec[4][1],
		float output[4][1]) {
	output[0][0] = m[0][0]*vec[0][0] + m[0][1]*vec[1][0] + m[0][2]*vec[2][0] + m[0][3]*vec[3][0];
	output[1][0] = m[1][0]*vec[0][0] + m[1][1]*vec[1][0] + m[1][2]*vec[2][0] + m[1][3]*vec[3][0];
	output[2][0] = m[2][0]*vec[0][0] + m[2][1]*vec[1][0] + m[2][2]*vec[2][0] + m[2][3]*vec[3][0];
	output[3][0] = m[3][0]*vec[0][0] + m[3][1]*vec[1][0] + m[3][2]*vec[2][0] + m[3][3]*vec[3][0];
}

// This is the invert of a 4*4 matrix
int invertColumnMajor(float n[4][4], float invOut[4][4])
{
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


// Temporary development groups
LOG_GROUP_START(kalman_states)
	LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
	LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
	LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
	LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)

	// Stock log groups
LOG_GROUP_START(MalNodeDetect)
	LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
	LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])

	STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
	PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
	PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)
