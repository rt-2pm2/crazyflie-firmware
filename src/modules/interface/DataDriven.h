/*
 * DataDriven.h
 *
 * Code generation for model "DataDriven".
 *
 * Model version              : 1.15
 * Simulink Coder version : 8.11 (R2016b) 25-Aug-2016
 * C source code generated on : Fri Aug 30 16:38:21 2019
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objective: Debugging
 * Validation result: All passed
 */

#ifndef RTW_HEADER_DataDriven_h_
#define RTW_HEADER_DataDriven_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef DataDriven_COMMON_INCLUDES_
# define DataDriven_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* DataDriven_COMMON_INCLUDES_ */

#include "DataDriven_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T State[3];                     /* '<S1>/Observer3' */
  real_T Alpha;                        /* '<S1>/Controller Variable Degree1' */
  real_T Beta;                         /* '<S1>/Controller Variable Degree1' */
  real_T u;                            /* '<S1>/Controller Variable Degree1' */
  real_T e;                            /* '<S1>/Controller Variable Degree1' */
  real_T Tmin;                         /* '<S1>/Controller Variable Degree1' */
} B_DataDriven_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T yold[5];                      /* '<S1>/Observer3' */
  real_T Step;                         /* '<S1>/Observer3' */
  real_T O[15];                        /* '<S1>/Observer3' */
  real_T StateEstimate[15];            /* '<S1>/Controller Variable Degree1' */
  real_T Step_c;                       /* '<S1>/Controller Variable Degree1' */
  real_T Uold[5];                      /* '<S1>/Controller Variable Degree1' */
  real_T Memory[5];                    /* '<S1>/Controller Variable Degree1' */
  real_T BetaBar[5];                   /* '<S1>/Controller Variable Degree1' */
  real_T AlphaBar[5];                  /* '<S1>/Controller Variable Degree1' */
  real_T K[2];                         /* '<S1>/Controller Variable Degree1' */
  boolean_T O_not_empty;               /* '<S1>/Observer3' */
  boolean_T K_not_empty;               /* '<S1>/Controller Variable Degree1' */
} DW_DataDriven_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T Measurements;                 /* '<Root>/Measurements' */
} ExtU_DataDriven_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T AlphaEstimate;                /* '<Root>/AlphaEstimate' */
  real_T BetaEstimate;                 /* '<Root>/BetaEstimate' */
  real_T Output;                       /* '<Root>/Output' */
  real_T Error;                        /* '<Root>/Error' */
  real_T MinimumT;                     /* '<Root>/MinimumT' */
} ExtY_DataDriven_T;

/* Parameters (auto storage) */
struct P_DataDriven_T_ {
  real_T SamplingTimeObserver_Value;   /* Expression: 0.0032
                                        * Referenced by: '<S1>/SamplingTimeObserver'
                                        */
  real_T Tracking_Value;               /* Expression: 1.2
                                        * Referenced by: '<S1>/Tracking'
                                        */
  real_T ControllerGain_Value;         /* Expression: 500
                                        * Referenced by: '<S1>/ControllerGain'
                                        */
  real_T ObserverGainAlpha_Value;      /* Expression: 2
                                        * Referenced by: '<S1>/ObserverGainAlpha'
                                        */
  real_T ObserverGainBeta_Value;       /* Expression: 3
                                        * Referenced by: '<S1>/ObserverGainBeta'
                                        */
  real_T SamplingTimeController_Value; /* Expression: 0.016
                                        * Referenced by: '<S1>/SamplingTimeController'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_DataDriven_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    struct {
      uint8_T TID[2];
    } TaskCounters;

    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern P_DataDriven_T DataDriven_P;

/* Block signals (auto storage) */
extern B_DataDriven_T DataDriven_B;

/* Block states (auto storage) */
extern DW_DataDriven_T DataDriven_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_DataDriven_T DataDriven_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_DataDriven_T DataDriven_Y;

/* Model entry point functions */
extern void DataDriven_initialize(void);
extern void DataDriven_step(void);
extern void DataDriven_terminate(void);

/* Real-time Model object */
extern RT_MODEL_DataDriven_T *const DataDriven_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('CodeTest/DataDriven')    - opens subsystem CodeTest/DataDriven
 * hilite_system('CodeTest/DataDriven/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CodeTest'
 * '<S1>'   : 'CodeTest/DataDriven'
 * '<S2>'   : 'CodeTest/DataDriven/Controller Variable Degree1'
 * '<S3>'   : 'CodeTest/DataDriven/Observer3'
 */
#endif                                 /* RTW_HEADER_DataDriven_h_ */
