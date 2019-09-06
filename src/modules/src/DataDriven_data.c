/*
 * DataDriven_data.c
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

#include "DataDriven.h"
#include "DataDriven_private.h"

/* Block parameters (auto storage) */
P_DataDriven_T DataDriven_P = {
  0.0032,                              /* Expression: 0.0032
                                        * Referenced by: '<S1>/SamplingTimeObserver'
                                        */
  1.2,                                 /* Expression: 1.2
                                        * Referenced by: '<S1>/Tracking'
                                        */
  500.0,                               /* Expression: 500
                                        * Referenced by: '<S1>/ControllerGain'
                                        */
  2.0,                                 /* Expression: 2
                                        * Referenced by: '<S1>/ObserverGainAlpha'
                                        */
  3.0,                                 /* Expression: 3
                                        * Referenced by: '<S1>/ObserverGainBeta'
                                        */
  0.016                                /* Expression: 0.016
                                        * Referenced by: '<S1>/SamplingTimeController'
                                        */
};
