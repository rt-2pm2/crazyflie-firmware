/*
 * DataDriven_private.h
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

#ifndef RTW_HEADER_DataDriven_private_h_
#define RTW_HEADER_DataDriven_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               (&(rtm)->Timing.taskTime0)
#endif
#endif                                 /* RTW_HEADER_DataDriven_private_h_ */
