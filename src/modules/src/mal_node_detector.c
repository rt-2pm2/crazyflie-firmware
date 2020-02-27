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

#define MODE_Y

#include "debug.h"

#define Xind (0)
#define Yind (1)
#define Zind (2)
#define DEG2RAD (3.14f / 180.0f)

#define BUFF_LENGHT (2)
#define NUM_MAX_MALICIOUS (5)
#define NUM_MEAS (3)
#define NUM_EQS (6)
#define STATE_DIM (6)

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
#define TASKPERIOD (50) 

/*
 * Variables Identifying the 2 good anchors
 */
static uint8_t base0 = 5;
static uint8_t base1 = 6;
static uint8_t base2 = 7;

static bool firstTime = true;
static bool updated_meas = false;

static float rel_threshold = 0.3;
static float abs_threshold = 0.1;
static bool attack_detected = false;

static uint8_t counter_thr = 3;

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
static anchor_data_t AnchorMeasBuffer[BUFF_LENGHT][NUM_ANCHORS] ;


/*
 * Actuation data and respective buffer
 * The actuation is considere the Forces along x,y and z
 */
static float Actuation[3];
static float UBuffer[BUFF_LENGHT][3];

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
static uint8_t det_counters[NUM_MAX_MALICIOUS];

// Index for the buffers

// Residuals for the detection
static float residual[NUM_MAX_MALICIOUS];

// Estimate
static float estimate[STATE_DIM];


// ===========================================================
// ===========================================================


// Internal Functions
static void MND_Task(void* parameters);
STATIC_MEM_TASK_ALLOC(MND_Task, 4 * configMINIMAL_STACK_SIZE);

static void initAnchorsPosition();
static bool averaging();
static void buildCMatrix(float CMat[NUM_MEAS][STATE_DIM], int i);
static int buildInvObs(float InvObs[STATE_DIM][NUM_EQS], const float C[NUM_MEAS][STATE_DIM], int i, float delta);
static bool generate_b(int num, const anchor_data_t AnchorData[NUM_ANCHORS], float b_vect[NUM_ANCHORS]);
static void prep_actuation(float T, float r, float p, float Fxyz[3]);
//static bool rule(float residual[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]);
static bool rule_vect(const float est[STATE_DIM][NUM_MAX_MALICIOUS], float v[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]);
float square_norm(const float v[3]);



bool healty_sensor_data(anchor_data_t AnchorMeasBuffer[BUFF_LENGHT][NUM_ANCHORS]) {
	bool output = true;
	for (int k = 0; k < BUFF_LENGHT; k++) {
		for (int i = 0; i < NUM_ANCHORS; i++) {
			if (AnchorMeasBuffer[k][i].data.distance < 0.0001f) {
				DEBUG_PRINT("[%d] Reading zero or negative distance!\n", i);
				output = false;
				return output;
			}
		}
	}
	return output;
}



// ===========================================================


static void MND_Task(void* parameters) {
	systemWaitStart();

	uint32_t current_tick = xTaskGetTickCount();
	uint32_t previous_tick = xTaskGetTickCount();

	int8_t curr_row = 0;
	bool data_health = true;

	while (true) {
		xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

		xSemaphoreTake(dataMutex, portMAX_DELAY);	

		memcpy(AnchorMeasBuffer[curr_row], AnchorMeas, sizeof(AnchorMeas));	
		memcpy(UBuffer[curr_row], Actuation, 3 * sizeof(float));		

		if (firstTime || pdFALSE || !updated_meas) {
			firstTime = false;
			xSemaphoreGive(dataMutex);
		} else { 
			updated_meas = false;
			// Allocate variables
			float b_vect00[NUM_ANCHORS] = {0};
			float b_vect01[NUM_ANCHORS] = {0};
			float b_vect02[NUM_ANCHORS] = {0};

			float b_vect10[NUM_ANCHORS] = {0};
			float b_vect11[NUM_ANCHORS] = {0};
			float b_vect12[NUM_ANCHORS] = {0};

			current_tick  = xTaskGetTickCount();
			float delta = T2S(current_tick - previous_tick);

			data_health = healty_sensor_data(AnchorMeasBuffer);

			if (delta > 0.0f && data_health) {
				previous_tick = current_tick;
				
				// Generate measurements from the previous data
				generate_b(base0, AnchorMeasBuffer[(curr_row + 1) % 2], b_vect00); 
				generate_b(base1, AnchorMeasBuffer[(curr_row + 1) % 2], b_vect01);
				generate_b(base2, AnchorMeasBuffer[(curr_row + 1) % 2], b_vect02);

				// Processing the current measurement
				generate_b(base0, AnchorMeasBuffer[curr_row], b_vect10);
				generate_b(base1, AnchorMeasBuffer[curr_row], b_vect11);
				generate_b(base2, AnchorMeasBuffer[curr_row], b_vect12);


				float recons[STATE_DIM][NUM_MAX_MALICIOUS] = {0};  // Used to store the reconstruct of each beacon

				for (uint8_t i = 0; i < NUM_MAX_MALICIOUS; i++) {

					if (i == base0 || i == base1 || i == base2)
						continue;

					float Y[NUM_EQS];
					float ObsInv[STATE_DIM][NUM_EQS];
					float C[NUM_MEAS][STATE_DIM] = {0};	

					float U[3] = {
						UBuffer[(curr_row + 1) % 2][Xind] / CF_MASS,
						UBuffer[(curr_row + 1) % 2][Yind] / CF_MASS,
						UBuffer[(curr_row + 1) % 2][Zind] / CF_MASS - GRAVITY_MAGNITUDE
					};

					buildCMatrix(C, i);

					float dt2_2 = delta * delta / 2.0f;
					Y[0] = b_vect00[i] + C[0][0] * dt2_2 * U[Xind] + C[0][2] * dt2_2 * U[Yind] + C[0][4] * dt2_2 * U[Zind];
					Y[1] = b_vect01[i] + C[1][0] * dt2_2 * U[Xind] + C[1][2] * dt2_2 * U[Yind] + C[1][4] * dt2_2 * U[Zind];
					Y[2] = b_vect02[i] + C[2][0] * dt2_2 * U[Xind] + C[1][2] * dt2_2 * U[Yind] + C[2][4] * dt2_2 * U[Zind];

					Y[3] = b_vect10[i];
					Y[4] = b_vect11[i];
					Y[5] = b_vect12[i];

					buildInvObs(ObsInv, C, i, delta);

					arm_matrix_instance_f32 ObsInv_;
					arm_matrix_instance_f32 Y_;
					arm_matrix_instance_f32 estimate_;

					arm_mat_init_f32(&ObsInv_, NUM_EQS, NUM_EQS, (float *)&ObsInv[0][0]);
					arm_mat_init_f32(&Y_, NUM_EQS, 1, Y);
					arm_mat_init_f32(&estimate_, STATE_DIM, 1, estimate);

					arm_status op_status = arm_mat_mult_f32(&ObsInv_, &Y_, &estimate_);

					if (op_status != ARM_MATH_SUCCESS) {
						DEBUG_PRINT("Error in matrix operation!\n");
					}

					for (int j = 0; j < STATE_DIM; j++) {
						recons[j][i] = estimate[j];
					}
				}

				float W[6] = {100,1,100,1,100,1};

				//DEBUG_PRINT("[%2.2f %2.2f %2.2f \n", (double)recons[0][0], (double)recons[2][0], (double)recons[4][0]);
				for (int j = 0; j < NUM_MAX_MALICIOUS; j++) {
					residual[j] = 0;
					for (int k = 0; k < STATE_DIM; k++) {
						residual[j] += W[k] * recons[k][j];
					}
				}

				//attack_detected = rule(residual, outcome);
				attack_detected = rule_vect(recons, residual, outcome);
			}
		}	

		curr_row = (curr_row + 1) % 2;
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

	updated_meas = true;	

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
 * Args:
 * 	num: Index of the anchor used as a reference
 * 	AnchorData: Array of distance measurements
 * 	b_vect: Manipulated measurements
 * 	A x = b
 */
bool generate_b(int num, const anchor_data_t AnchorData[NUM_ANCHORS], float b_vect[NUM_ANCHORS]) {
	// Temp variables
	float meas2[NUM_ANCHORS];
	float meas2_d[NUM_ANCHORS];

	// Square the distances from the anchors and put the 
	// result in the temporary vector meas2
	for (int i = 0; i < NUM_ANCHORS; i++) {
		if (AnchorData[i].data.distance < 0.0f) {
			DEBUG_PRINT("[%d] Reading negative distance!\n", i);
			return false;
		}

		meas2[i] = powf(AnchorData[i].data.distance, 2);

		if (meas2[i] < 0.0001f) {
			DEBUG_PRINT("[%d] Reading 0 distance!: %2.1f \n", i, (double)AnchorData[i].data.distance);
			return false;
		}
	}

	// Select the reference anchor
	float anchor = meas2[num];

	for (int i = 0; i < NUM_ANCHORS; i++) {
		meas2_d[i] = anchor - meas2[i];
		meas2_d[i] = meas2_d[i] + square_norm(MND_Data.APos[i]);
		meas2_d[i] = meas2_d[i] - square_norm(MND_Data.APos[num]);

		// Update the output data
		b_vect[i] = meas2_d[i];
	}
	return true;
}  

arm_status eval_pseudoinv(const arm_matrix_instance_f32* pMat, arm_matrix_instance_f32* Pseudo) {
	arm_status op_status;

	float TransMat[STATE_DIM * NUM_EQS];
	arm_matrix_instance_f32 TransMat_arm = {STATE_DIM, STATE_DIM, TransMat};

	// Temporary Mat 1
	float TempNxNx[STATE_DIM * STATE_DIM];
	arm_matrix_instance_f32 TempNxNx_arm = {STATE_DIM, STATE_DIM, TempNxNx};

	// Temporary Mat 2
	float TempNxNx2[STATE_DIM * STATE_DIM];
	arm_matrix_instance_f32 TempNxNx2_arm = {STATE_DIM, STATE_DIM, TempNxNx2};


	arm_mat_trans_f32(pMat, &TransMat_arm); // O'

	arm_mat_mult_f32(&TransMat_arm, pMat, &TempNxNx_arm); // (O' x O)

	op_status = arm_mat_inverse_f32(&TempNxNx_arm, &TempNxNx2_arm);

	arm_mat_mult_f32(&TempNxNx2_arm, &TransMat_arm, Pseudo);  // (O' x O)^-1 x O' = Pseudo inverse 

	return op_status;
}

int buildInvObs(float InvObs[STATE_DIM][NUM_EQS], const float C[NUM_MEAS][STATE_DIM], int i, float delta) {

	static float O[NUM_EQS * STATE_DIM] = {0.0f};
	// C * Ab
	O[0] = C[0][0];
	O[1] = -delta * C[0][0];
	O[2] = C[0][2];
	O[3] = -delta * C[0][2];
	O[4] = C[0][4];
	O[5] = -delta * C[0][4];
	
	O[6+0] = C[1][0];
	O[6+1] = -delta * C[1][0];
	O[6+2] = C[1][2];
	O[6+3] = -delta * C[1][2];
	O[6+4] = C[1][4];
	O[6+5] = -delta * C[1][4];

	O[12+0] = C[2][0];
	O[12+1] = -delta * C[2][0];
	O[12+2] = C[2][2];
	O[12+3] = -delta * C[2][2];
	O[12+4] = C[2][4];
	O[12+5] = -delta * C[2][4];
	
	// C
	O[18+0] = C[0][0];
	O[18+2] = C[0][2];
	O[18+4] = C[0][4];
	
	O[24+0] = C[1][0];
	O[24+2] = C[1][2];
	O[24+4] = C[1][4];

	O[30+0] = C[2][0];
	O[30+2] = C[2][2];
	O[30+4] = C[2][4];

	arm_matrix_instance_f32 O_;
	arm_matrix_instance_f32 Oinv_;
	arm_mat_init_f32(&O_, STATE_DIM, STATE_DIM, (float32_t*)O);
	arm_mat_init_f32(&Oinv_, STATE_DIM, STATE_DIM, (float *) &InvObs[0][0]);

	// The standard inversion was problematic, thus I perform
	// pseudo inversion which seems more numerically stable.
	//arm_status op_status = arm_mat_inverse_f32(&O_, &Oinv_);		
	arm_status op_status = eval_pseudoinv(&O_, &Oinv_);

	if (op_status != ARM_MATH_SUCCESS) {		
		DEBUG_PRINT("Error in matrix inversion! [CODE = %d]\n", op_status);
		DEBUG_PRINT("dt = %2.2f \n", (double)delta);
	
		return -1;
	}

	return 0;
}


bool rule_vect(const float est[STATE_DIM][NUM_MAX_MALICIOUS], float e_norm[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]) {
	float p_avg[3] = {0};
	//float e_norm[NUM_MAX_MALICIOUS] = {0};
	int8_t o[NUM_MAX_MALICIOUS]; 
	bool ordered = false;
	bool detected = false;

	for (int8_t i = 0; i < NUM_MAX_MALICIOUS; i++) {
		o[i] = i;
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < NUM_MAX_MALICIOUS; j++) {
			p_avg[i] += est[i * 2][j];
		}
		p_avg[i] /= NUM_MAX_MALICIOUS;
	}
	
	for (int i = 0; i < NUM_MAX_MALICIOUS; i++) {
		e_norm[i] = powf(est[0][i] - p_avg[0], 2) + powf(est[2][i] - p_avg[1], 2) + powf(est[4][i] - p_avg[2], 2);
	}

	/*
	DEBUG_PRINT("avg = [%3.2f, %3.2f, %3.2f]\n", (double)p_avg[0], (double)p_avg[1], (double)p_avg[2]);
	DEBUG_PRINT("0 [%3.2f, %3.2f, %3.2f]\n", (double)est[0][0], (double)est[2][0], (double)est[4][0]);
	DEBUG_PRINT("1 [%3.2f, %3.2f, %3.2f]\n", (double)est[0][1], (double)est[2][1], (double)est[4][1]);
	DEBUG_PRINT("2 [%3.2f, %3.2f, %3.2f]\n", (double)est[0][2], (double)est[2][2], (double)est[4][2]);
	DEBUG_PRINT("err = [%3.2f, %3.2f, %3.2f]\n", (double)e_norm[0], (double)e_norm[1], (double)e_norm[2]);
	*/
	while (!ordered) {
		ordered = true;
		for (int i = 0; i < NUM_MAX_MALICIOUS - 1; i++) {
			if (e_norm[i] < e_norm[i+1]) {
				ordered = false;
				float temp = e_norm[i];
				int8_t temp_d = o[i];
	
				// Swapping
				e_norm[i] = e_norm[i + 1];
				o[i] = o[i + 1];

				e_norm[i + 1] = temp;
				o[i + 1] = temp_d;
			}
		}
	}

	// Compute the difference between the first 2  residuals to understand 
	// if there was actually an attack.
	//float diff_residual = fabsf(e_norm[0] - e_norm[1]) / e_norm[0];
	if (e_norm[0] < abs_threshold) {
		// Reset the array to a definite value to indicate 
		// that there was no malicious attack.
		for (int i = 0; i < NUM_MAX_MALICIOUS; i++) {
			outcome[i] = -1;
			det_counters[i] = 0;
		}
		detected = false;
	} else {
		for (int i = 0; i < NUM_MAX_MALICIOUS; i++) {
			outcome[i] = o[i];
			if (i > 0) {
				// Reset the counter for the nodes that are not in the first position
				det_counters[o[i]] = 0;
			}
		}

		// Update the counter for the identified malicious node
		det_counters[o[0]]++;

		// If we got more that a given amount of detection trigger the alarm
		if (det_counters[o[0]] > counter_thr) {
			detected = true;
		}
	}

	return detected;
}


//find out the guy that deviates the average most
/*
bool rule(float residual[NUM_MAX_MALICIOUS], int8_t outcome[NUM_MAX_MALICIOUS]) {
	float sum = 0;
	bool ordered = false;
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
		for (int i = 0; i < NUM_MAX_MALICIOUS - 1; i++) {
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
	diff_residual = fabsf(residual[0] - residual[1]) / residual[0];
	if (diff_residual < rel_threshold || residual[0] < abs_threshold) {
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
*/

static void initAnchorsPosition() {
	MND_Data.APos[0][0] = 2.60f;
	MND_Data.APos[0][1] = -2.05f;
	MND_Data.APos[0][2] = 0.0f;

	MND_Data.APos[1][0] = -2.66f;
	MND_Data.APos[1][1] = -2.05f;
	MND_Data.APos[1][2] = 0.0f;

	MND_Data.APos[2][0] = -2.66f;
	MND_Data.APos[2][1] = 1.47f;
	MND_Data.APos[2][2] = 0.0f;

	MND_Data.APos[3][0] = 2.60f;
	MND_Data.APos[3][1] = 1.47f;
	MND_Data.APos[3][2] = 0.74f;

	MND_Data.APos[4][0] = -0.12f;
	MND_Data.APos[4][1] = -2.10f;
	MND_Data.APos[4][2] = 1.90f;

	MND_Data.APos[5][0] = 1.62f;
	MND_Data.APos[5][1] = -2.76f;
	MND_Data.APos[5][2] = 1.40f;

	MND_Data.APos[6][0] = -0.99f;
	MND_Data.APos[6][1] = 1.45f;
	MND_Data.APos[6][2] = 0.76f;

	MND_Data.APos[7][0] = -2.39f;
	MND_Data.APos[7][1] = 0.0f;
	MND_Data.APos[7][2] = 0.78f;
}

static void buildCMatrix(float CMat[NUM_MEAS][STATE_DIM], int i) {
	CMat[0][0] = 2.0f * (MND_Data.APos[i][Xind] - MND_Data.APos[base0][Xind]);
	CMat[0][2] = 2.0f * (MND_Data.APos[i][Yind] - MND_Data.APos[base0][Yind]);
	CMat[0][4] = 2.0f * (MND_Data.APos[i][Zind] - MND_Data.APos[base0][Zind]);
	
	CMat[1][0] = 2.0f * (MND_Data.APos[i][Xind] - MND_Data.APos[base1][Xind]);
	CMat[1][2] = 2.0f * (MND_Data.APos[i][Yind] - MND_Data.APos[base1][Yind]);
	CMat[1][4] = 2.0f * (MND_Data.APos[i][Zind] - MND_Data.APos[base1][Zind]);

	CMat[2][0] = 2.0f * (MND_Data.APos[i][Xind] - MND_Data.APos[base2][Xind]);
	CMat[2][2] = 2.0f * (MND_Data.APos[i][Yind] - MND_Data.APos[base2][Yind]);
	CMat[2][4] = 2.0f * (MND_Data.APos[i][Zind] - MND_Data.APos[base2][Zind]);

	return;
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

	// Initialize the C matrix


	// Initialize the value of the outcomes
	for (i = 0; i < NUM_MAX_MALICIOUS; i++) {
		outcome[i] = -1;
	}
	
	STATIC_MEM_TASK_CREATE(
			MND_Task,
			MND_Task, MND_TASK_NAME,
			NULL, MND_TASK_PRI);
}

/**
 * Helper function
 */
float square_norm(const float v[3]) {
	float output;
	output = powf(v[Xind], 2) +
		powf(v[Yind], 2) +
		powf(v[Zind], 2);

	return output;
}

// Temporary development groups
LOG_GROUP_START(mnd_log)
	LOG_ADD(LOG_INT8, outcome0, &outcome[0])
	LOG_ADD(LOG_UINT8, enable, &activated_mnd)
	LOG_ADD(LOG_FLOAT, residual0, &residual[0])
	LOG_ADD(LOG_FLOAT, residual1, &residual[1])
LOG_GROUP_STOP(mnd_log)

LOG_GROUP_START(mnd_est_log)
	LOG_ADD(LOG_FLOAT, x_est, &estimate[0])
	LOG_ADD(LOG_FLOAT, y_est, &estimate[2])
	LOG_ADD(LOG_FLOAT, z_est, &estimate[4])
LOG_GROUP_STOP(mnd_est_log)

/*
LOG_GROUP_START(mnd_dgb_log)
	LOG_ADD(LOG_FLOAT, thrustAvg, &MND_Data.thrustAverage)
	LOG_ADD(LOG_FLOAT, rollAvg, &MND_Data.rollAverage)
	LOG_ADD(LOG_FLOAT, pitchAvg, &MND_Data.pitchAverage)
LOG_GROUP_STOP(mnd_dbg_log)
*/

PARAM_GROUP_START(mnd_param)
	PARAM_ADD(PARAM_UINT8, activate, &activated_mnd)
	PARAM_ADD(PARAM_UINT8, base0, &base0)
	PARAM_ADD(PARAM_UINT8, base1, &base1)
	PARAM_ADD(PARAM_UINT8, base2, &base2)
	PARAM_ADD(PARAM_FLOAT, rel_threshold, &rel_threshold)
	PARAM_ADD(PARAM_FLOAT, abs_threshold, &abs_threshold)
	PARAM_ADD(PARAM_UINT8, det_counter, &counter_thr)
PARAM_GROUP_STOP(mnd_param)
