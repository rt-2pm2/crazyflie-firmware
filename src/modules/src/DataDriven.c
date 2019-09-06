/*
 * DataDriven.c
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

/* Block signals (auto storage) */
B_DataDriven_T DataDriven_B;

/* Block states (auto storage) */
DW_DataDriven_T DataDriven_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_DataDriven_T DataDriven_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_DataDriven_T DataDriven_Y;

/* Real-time model */
RT_MODEL_DataDriven_T DataDriven_M_;
RT_MODEL_DataDriven_T *const DataDriven_M = &DataDriven_M_;

/* Forward declaration for local functions */
static void DataDriven_circshift(real_T a[5]);
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (DataDriven_M->Timing.TaskCounters.TID[1])++;
  if ((DataDriven_M->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.016s, 0.0s] */
    DataDriven_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Function for MATLAB Function: '<S1>/Controller Variable Degree1' */
static void DataDriven_circshift(real_T a[5])
{
  real_T b_a[5];
  int16_T i;
  for (i = 0; i < 5; i++) {
    b_a[i] = a[i];
  }

  b_a[4] = b_a[3];
  b_a[3] = b_a[2];
  b_a[2] = b_a[1];
  b_a[1] = b_a[0];
  b_a[0] = a[4];
  for (i = 0; i < 5; i++) {
    a[i] = b_a[i];
  }
}

/* Model step function */
void DataDriven_step(void)
{
  real_T a[5];
  real_T y[9];
  real_T x[9];
  int16_T p2;
  int16_T p3;
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int16_T itmp;
  real_T Tracking;
  real_T Beta;
  int16_T i;
  real_T y_0[15];
  real_T Track_idx_0;
  real_T Track_idx_1;
  real_T TrackOld_idx_0;
  real_T TrackOld_idx_1;
  real_T tmp;

  /* MATLAB Function: '<S1>/Observer3' incorporates:
   *  Constant: '<S1>/SamplingTimeObserver'
   *  Inport: '<Root>/Measurements'
   */
  /* MATLAB Function 'DataDriven/Observer3': '<S3>:1' */
  /* %% Sampling time step being used, needs to be changed if you change the */
  /* %% Matlab Function's sampling time */
  /* '<S3>:1:5' T=SamplingTime; */
  /* '<S3>:1:6' if isempty(yold) */
  /* '<S3>:1:9' if isempty(Step) */
  /* '<S3>:1:12' if isempty(z) */
  /* '<S3>:1:15' if isempty(O) */
  if (!DataDriven_DW.O_not_empty) {
    /* '<S3>:1:16' O=[ 1,    0,         0; */
    /* '<S3>:1:17'  1,   -T,     T^2/2; */
    /* '<S3>:1:18'  1, -2*T,     2*T^2; */
    /* '<S3>:1:19'  1, -3*T, (9*T^2)/2; */
    /* '<S3>:1:20'  1, -4*T,     8*T^2]; */
    DataDriven_DW.O[0] = 1.0;
    DataDriven_DW.O[5] = 0.0;
    DataDriven_DW.O[10] = 0.0;
    DataDriven_DW.O[1] = 1.0;
    DataDriven_DW.O[6] = -DataDriven_P.SamplingTimeObserver_Value;
    DataDriven_DW.O[11] = DataDriven_P.SamplingTimeObserver_Value *
      DataDriven_P.SamplingTimeObserver_Value / 2.0;
    DataDriven_DW.O[2] = 1.0;
    DataDriven_DW.O[7] = -2.0 * DataDriven_P.SamplingTimeObserver_Value;
    DataDriven_DW.O[12] = DataDriven_P.SamplingTimeObserver_Value *
      DataDriven_P.SamplingTimeObserver_Value * 2.0;
    DataDriven_DW.O[3] = 1.0;
    DataDriven_DW.O[8] = -3.0 * DataDriven_P.SamplingTimeObserver_Value;
    DataDriven_DW.O[13] = DataDriven_P.SamplingTimeObserver_Value *
      DataDriven_P.SamplingTimeObserver_Value * 9.0 / 2.0;
    DataDriven_DW.O[4] = 1.0;
    DataDriven_DW.O[9] = -4.0 * DataDriven_P.SamplingTimeObserver_Value;
    DataDriven_DW.O[14] = DataDriven_P.SamplingTimeObserver_Value *
      DataDriven_P.SamplingTimeObserver_Value * 8.0;
    DataDriven_DW.O_not_empty = true;
  }

  /* %% Update step */
  /* '<S3>:1:23' Step=Step+1; */
  DataDriven_DW.Step++;

  /* '<S3>:1:25' yold=circshift(yold,1); */
  for (i = 0; i < 5; i++) {
    a[i] = DataDriven_DW.yold[i];
  }

  a[4] = a[3];
  a[3] = a[2];
  a[2] = a[1];
  a[1] = a[0];
  a[0] = DataDriven_DW.yold[4];
  for (i = 0; i < 5; i++) {
    DataDriven_DW.yold[i] = a[i];
  }

  /* '<S3>:1:26' yold(1)=Measurements; */
  DataDriven_DW.yold[0] = DataDriven_U.Measurements;

  /* '<S3>:1:27' State=zeros(3,1); */
  DataDriven_B.State[0] = 0.0;
  DataDriven_B.State[1] = 0.0;
  DataDriven_B.State[2] = 0.0;

  /*  Amount of steps required to have enough measurements to do estimation */
  /* '<S3>:1:30' if Step >=7 */
  if (DataDriven_DW.Step >= 7.0) {
    /* %% Estimation */
    /* '<S3>:1:35' State=inv(O'*O)*O'*yold; */
    for (i = 0; i < 3; i++) {
      for (p2 = 0; p2 < 3; p2++) {
        y[i + 3 * p2] = 0.0;
        for (p3 = 0; p3 < 5; p3++) {
          absx11 = y[3 * p2 + i];
          absx11 += DataDriven_DW.O[5 * i + p3] * DataDriven_DW.O[5 * p2 + p3];
          y[i + 3 * p2] = absx11;
        }
      }
    }

    memcpy(&x[0], &y[0], 9U * sizeof(real_T));
    i = 0;
    p2 = 3;
    p3 = 6;
    absx11 = fabs(y[0]);
    absx21 = fabs(y[1]);
    absx31 = fabs(y[2]);
    if ((absx21 > absx11) && (absx21 > absx31)) {
      i = 3;
      p2 = 0;
      x[0] = y[1];
      x[1] = y[0];
      x[3] = y[4];
      x[4] = y[3];
      x[6] = y[7];
      x[7] = y[6];
    } else {
      if (absx31 > absx11) {
        i = 6;
        p3 = 0;
        x[0] = y[2];
        x[2] = y[0];
        x[3] = y[5];
        x[5] = y[3];
        x[6] = y[8];
        x[8] = y[6];
      }
    }

    absx11 = x[1] / x[0];
    x[1] /= x[0];
    absx21 = x[2] / x[0];
    x[2] /= x[0];
    x[4] -= absx11 * x[3];
    x[5] -= absx21 * x[3];
    x[7] -= absx11 * x[6];
    x[8] -= absx21 * x[6];
    if (fabs(x[5]) > fabs(x[4])) {
      itmp = p2;
      p2 = p3;
      p3 = itmp;
      x[1] = absx21;
      x[2] = absx11;
      absx11 = x[4];
      x[4] = x[5];
      x[5] = absx11;
      absx11 = x[7];
      x[7] = x[8];
      x[8] = absx11;
    }

    absx11 = x[5] / x[4];
    x[5] /= x[4];
    x[8] -= absx11 * x[7];
    absx11 = (x[5] * x[1] - x[2]) / x[8];
    absx21 = -(x[7] * absx11 + x[1]) / x[4];
    y[i] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
    y[i + 1] = absx21;
    y[i + 2] = absx11;
    absx11 = -x[5] / x[8];
    absx21 = (1.0 - x[7] * absx11) / x[4];
    y[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
    y[p2 + 1] = absx21;
    y[p2 + 2] = absx11;
    absx11 = 1.0 / x[8];
    absx21 = -x[7] * absx11 / x[4];
    y[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
    y[p3 + 1] = absx21;
    y[p3 + 2] = absx11;
    for (i = 0; i < 3; i++) {
      DataDriven_B.State[i] = 0.0;
      for (p2 = 0; p2 < 5; p2++) {
        y_0[i + 3 * p2] = 0.0;
        absx11 = y_0[3 * p2 + i];
        absx11 += y[i] * DataDriven_DW.O[p2];
        y_0[i + 3 * p2] = absx11;
        absx11 = y_0[3 * p2 + i];
        absx11 += y[i + 3] * DataDriven_DW.O[p2 + 5];
        y_0[i + 3 * p2] = absx11;
        absx11 = y_0[3 * p2 + i];
        absx11 += y[i + 6] * DataDriven_DW.O[p2 + 10];
        y_0[i + 3 * p2] = absx11;
        DataDriven_B.State[i] += y_0[3 * p2 + i] * DataDriven_DW.yold[p2];
      }
    }
  }

  /* End of MATLAB Function: '<S1>/Observer3' */
  if (DataDriven_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S1>/Controller Variable Degree1' incorporates:
     *  Constant: '<S1>/ControllerGain'
     *  Constant: '<S1>/ObserverGainAlpha'
     *  Constant: '<S1>/ObserverGainBeta'
     *  Constant: '<S1>/SamplingTimeController'
     *  Constant: '<S1>/Tracking'
     */
    Tracking = DataDriven_P.Tracking_Value;
    absx11 = DataDriven_P.ObserverGainAlpha_Value;
    absx21 = DataDriven_P.ObserverGainBeta_Value;
    absx31 = DataDriven_P.SamplingTimeController_Value;

    /* MATLAB Function 'DataDriven/Controller Variable Degree1': '<S2>:1' */
    /* %% Sampling time step being used, needs to be changed if you change the */
    /* %% Matlab Function's sampling time */
    /* '<S2>:1:4' L=Gain; */
    /* '<S2>:1:5' T=SamplingTime; */
    /* '<S2>:1:7' if isempty(StateEstimate) */
    /* '<S2>:1:10' if isempty(Uold) */
    /* '<S2>:1:13' if isempty(Memory) */
    /* '<S2>:1:16' if isempty(Step) */
    /* '<S2>:1:19' if isempty(BetaBar) */
    /* '<S2>:1:23' if isempty(AlphaBar) */
    /* '<S2>:1:27' if isempty(K) */
    if (!DataDriven_DW.K_not_empty) {
      /* '<S2>:1:28' K=[ -L^2, (T*L^2)/2 - 2*L]; */
      DataDriven_DW.K[0] = -(DataDriven_P.ControllerGain_Value *
        DataDriven_P.ControllerGain_Value);
      DataDriven_DW.K[1] = DataDriven_P.ControllerGain_Value *
        DataDriven_P.ControllerGain_Value *
        DataDriven_P.SamplingTimeController_Value / 2.0 - 2.0 *
        DataDriven_P.ControllerGain_Value;
      DataDriven_DW.K_not_empty = true;
    }

    /* %% Update step */
    /* '<S2>:1:32' Step=Step+1; */
    DataDriven_DW.Step_c++;

    /* '<S2>:1:33' T=SamplingTime; */
    /* %% Samples is the amount of samples to use for the estimation */
    /* %% Level is the level of this observer in the cascade, starts at 1 for the first one */
    /* %% The measured state is stored in the variable yold */
    /* '<S2>:1:38' Memory=circshift(Memory,1); */
    DataDriven_circshift(DataDriven_DW.Memory);

    /* '<S2>:1:39' Memory(1)=Tracking; */
    DataDriven_DW.Memory[0] = Tracking;

    /* '<S2>:1:40' StateEstimate=circshift(StateEstimate,1,2); */
    for (p2 = 0; p2 < 3; p2++) {
      Tracking = DataDriven_DW.StateEstimate[p2 + 12];
      DataDriven_DW.StateEstimate[p2 + 12] = DataDriven_DW.StateEstimate[p2 + 9];
      DataDriven_DW.StateEstimate[p2 + 9] = DataDriven_DW.StateEstimate[p2 + 6];
      DataDriven_DW.StateEstimate[p2 + 6] = DataDriven_DW.StateEstimate[p2 + 3];
      DataDriven_DW.StateEstimate[p2 + 3] = DataDriven_DW.StateEstimate[p2];
      DataDriven_DW.StateEstimate[p2] = Tracking;
    }

    /* '<S2>:1:41' StateEstimate(:,1)=y; */
    DataDriven_DW.StateEstimate[0] = DataDriven_B.State[0];
    DataDriven_DW.StateEstimate[1] = DataDriven_B.State[1];
    DataDriven_DW.StateEstimate[2] = DataDriven_B.State[2];

    /* '<S2>:1:42' Uold=circshift(Uold,1); */
    DataDriven_circshift(DataDriven_DW.Uold);

    /* '<S2>:1:43' AlphaBar=circshift(AlphaBar,1); */
    DataDriven_circshift(DataDriven_DW.AlphaBar);

    /* '<S2>:1:44' BetaBar=circshift(BetaBar,1); */
    DataDriven_circshift(DataDriven_DW.BetaBar);

    /*  In the first steps the estimate is constant until we get enough samples */
    /* '<S2>:1:47' Alpha=0; */
    Tracking = 0.0;

    /* '<S2>:1:48' Beta=1; */
    Beta = 1.0;

    /* '<S2>:1:49' eaux=[Alpha-(-2);Beta-3]; */
    /* '<S2>:1:50' e=eaux'*[1/GainObserverAlpha 0 ; 0 1/GainObserverBeta]*eaux; */
    /* '<S2>:1:51' Tmin=1; */
    /* '<S2>:1:52' Uold(1)=0.1; */
    DataDriven_DW.Uold[0] = 0.1;

    /* '<S2>:1:53' u=Uold(1); */
    /*  gamma1=1/T; */
    /*  gamma2=u1/(T*u1 - T*u2); */
    /*  beta1=0; */
    /*  beta2=1/(T*u2^2 - T*u1*u2); */
    /* '<S2>:1:58' if Step>2 */
    if (DataDriven_DW.Step_c > 2.0) {
      /* %% Estimation */
      /* '<S2>:1:60' Track=[Memory(1);(Memory(1)-Memory(2))/T]; */
      Track_idx_0 = DataDriven_DW.Memory[0];
      Track_idx_1 = (DataDriven_DW.Memory[0] - DataDriven_DW.Memory[1]) / absx31;

      /* '<S2>:1:61' TrackOld=[Memory(2);(Memory(2)-Memory(3))/T]; */
      TrackOld_idx_0 = DataDriven_DW.Memory[1];
      TrackOld_idx_1 = (DataDriven_DW.Memory[1] - DataDriven_DW.Memory[2]) /
        absx31;

      /* '<S2>:1:62' AlphaBar(1)=AlphaBar(2)+GainObserverAlpha*(StateEstimate(2,1)-StateEstimate(2,2)-T*K*(StateEstimate(1:2,2)-TrackOld)); */
      tmp = absx31 * DataDriven_DW.K[0] * (DataDriven_DW.StateEstimate[3] -
        TrackOld_idx_0);
      tmp += absx31 * DataDriven_DW.K[1] * (DataDriven_DW.StateEstimate[4] -
        TrackOld_idx_1);
      DataDriven_DW.AlphaBar[0] = ((DataDriven_DW.StateEstimate[1] -
        DataDriven_DW.StateEstimate[4]) - tmp) * absx11 +
        DataDriven_DW.AlphaBar[1];

      /* '<S2>:1:63' BetaBar(1)=BetaBar(2)+GainObserverBeta*Uold(2)*(StateEstimate(2,1)-StateEstimate(2,2)-T*K*(StateEstimate(1:2,2)-TrackOld)); */
      tmp = absx31 * DataDriven_DW.K[0] * (DataDriven_DW.StateEstimate[3] -
        TrackOld_idx_0);
      tmp += absx31 * DataDriven_DW.K[1] * (DataDriven_DW.StateEstimate[4] -
        TrackOld_idx_1);
      DataDriven_DW.BetaBar[0] = ((DataDriven_DW.StateEstimate[1] -
        DataDriven_DW.StateEstimate[4]) - tmp) * (absx21 * DataDriven_DW.Uold[1])
        + DataDriven_DW.BetaBar[1];

      /* '<S2>:1:64' Alpha=AlphaBar(1); */
      Tracking = DataDriven_DW.AlphaBar[0];

      /* '<S2>:1:65' Beta=BetaBar(1); */
      Beta = DataDriven_DW.BetaBar[0];

      /* '<S2>:1:67' Uold(1)=1/Beta*(-Alpha+K*(StateEstimate(1:2,1)-Track)); */
      tmp = (DataDriven_DW.StateEstimate[0] - Track_idx_0) * DataDriven_DW.K[0];
      tmp += (DataDriven_DW.StateEstimate[1] - Track_idx_1) * DataDriven_DW.K[1];
      DataDriven_DW.Uold[0] = 1.0 / DataDriven_DW.BetaBar[0] *
        (-DataDriven_DW.AlphaBar[0] + tmp);

      /* '<S2>:1:68' if Step==4 */
      if (DataDriven_DW.Step_c == 4.0) {
        /* '<S2>:1:69' AlphaBar(1)=AlphaBar(2)+1/T*(StateEstimate(2,1)-StateEstimate(2,2)-T*K*(StateEstimate(1:2,2)-TrackOld)); */
        tmp = absx31 * DataDriven_DW.K[0] * (DataDriven_DW.StateEstimate[3] -
          TrackOld_idx_0);
        tmp += absx31 * DataDriven_DW.K[1] * (DataDriven_DW.StateEstimate[4] -
          TrackOld_idx_1);
        DataDriven_DW.AlphaBar[0] = ((DataDriven_DW.StateEstimate[1] -
          DataDriven_DW.StateEstimate[4]) - tmp) * (1.0 / absx31) +
          DataDriven_DW.AlphaBar[1];

        /* '<S2>:1:70' BetaBar(1)=BetaBar(2)+0*Uold(2)*(StateEstimate(2,1)-StateEstimate(2,2)-T*K*(StateEstimate(1:2,2)-TrackOld)); */
        tmp = absx31 * DataDriven_DW.K[0] * (DataDriven_DW.StateEstimate[3] -
          TrackOld_idx_0);
        tmp += absx31 * DataDriven_DW.K[1] * (DataDriven_DW.StateEstimate[4] -
          TrackOld_idx_1);
        DataDriven_DW.BetaBar[0] = ((DataDriven_DW.StateEstimate[1] -
          DataDriven_DW.StateEstimate[4]) - tmp) * (0.0 * DataDriven_DW.Uold[1])
          + DataDriven_DW.BetaBar[1];

        /* '<S2>:1:71' Uold(1)=2; */
        DataDriven_DW.Uold[0] = 2.0;
      } else {
        if (DataDriven_DW.Step_c == 5.0) {
          /* '<S2>:1:72' elseif Step==5 */
          /* '<S2>:1:73' AlphaBar(1)=AlphaBar(2)+Uold(3)/(T*Uold(3) - T*Uold(2))*(StateEstimate(2,1)-StateEstimate(2,2)-T*(AlphaBar(2)+BetaBar(2)*Uold(2))); */
          DataDriven_DW.AlphaBar[0] = ((DataDriven_DW.StateEstimate[1] -
            DataDriven_DW.StateEstimate[4]) - (DataDriven_DW.BetaBar[1] *
            DataDriven_DW.Uold[1] + DataDriven_DW.AlphaBar[1]) * absx31) *
            (DataDriven_DW.Uold[2] / (absx31 * DataDriven_DW.Uold[2] - absx31 *
              DataDriven_DW.Uold[1])) + DataDriven_DW.AlphaBar[1];

          /* '<S2>:1:74' BetaBar(1)=BetaBar(2)+1/(T*Uold(2) - T*Uold(3))*(StateEstimate(2,1)-StateEstimate(2,2)-T*(AlphaBar(2)+BetaBar(2)*Uold(2))); */
          DataDriven_DW.BetaBar[0] = ((DataDriven_DW.StateEstimate[1] -
            DataDriven_DW.StateEstimate[4]) - (DataDriven_DW.BetaBar[1] *
            DataDriven_DW.Uold[1] + DataDriven_DW.AlphaBar[1]) * absx31) * (1.0 /
            (absx31 * DataDriven_DW.Uold[1] - absx31 * DataDriven_DW.Uold[2])) +
            DataDriven_DW.BetaBar[1];

          /* '<S2>:1:75' Alpha=AlphaBar(1); */
          Tracking = DataDriven_DW.AlphaBar[0];

          /* '<S2>:1:76' Beta=BetaBar(1); */
          Beta = DataDriven_DW.BetaBar[0];

          /* '<S2>:1:78' Uold(1)=1/Beta*(-Alpha+K*(StateEstimate(1:2,1)-Track)); */
          tmp = (DataDriven_DW.StateEstimate[0] - Track_idx_0) *
            DataDriven_DW.K[0];
          tmp += (DataDriven_DW.StateEstimate[1] - Track_idx_1) *
            DataDriven_DW.K[1];
          DataDriven_DW.Uold[0] = 1.0 / DataDriven_DW.BetaBar[0] *
            (-DataDriven_DW.AlphaBar[0] + tmp);
        }
      }
    }

    /* '<S2>:1:86' u=Uold(1); */
    /* '<S2>:1:87' eaux=[Alpha-(-2);Beta-3]; */
    Track_idx_0 = Tracking - -2.0;
    Track_idx_1 = Beta - 3.0;

    /* '<S2>:1:88' e=eaux'*[1/GainObserverAlpha 0 ; 0 1/GainObserverBeta]*eaux; */
    /* '<S2>:1:89' Tmin=2/(GainObserverAlpha+GainObserverBeta*u^2); */
    DataDriven_B.Alpha = Tracking;
    DataDriven_B.Beta = Beta;
    DataDriven_B.u = DataDriven_DW.Uold[0];
    tmp = 1.0 / absx11;
    absx31 = 1.0 / absx21;
    tmp *= Track_idx_0;
    tmp += Track_idx_1 * 0.0;
    tmp *= Track_idx_0;
    Track_idx_0 *= 0.0;
    Track_idx_0 += Track_idx_1 * absx31;
    tmp += Track_idx_0 * Track_idx_1;
    DataDriven_B.e = tmp;
    DataDriven_B.Tmin = 2.0 / (DataDriven_DW.Uold[0] * DataDriven_DW.Uold[0] *
      absx21 + absx11);

    /* End of MATLAB Function: '<S1>/Controller Variable Degree1' */

    /* Outport: '<Root>/AlphaEstimate' */
    DataDriven_Y.AlphaEstimate = DataDriven_B.Alpha;

    /* Outport: '<Root>/BetaEstimate' */
    DataDriven_Y.BetaEstimate = DataDriven_B.Beta;

    /* Outport: '<Root>/Output' */
    DataDriven_Y.Output = DataDriven_B.u;

    /* Outport: '<Root>/Error' */
    DataDriven_Y.Error = DataDriven_B.e;

    /* Outport: '<Root>/MinimumT' */
    DataDriven_Y.MinimumT = DataDriven_B.Tmin;
  }


  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0032s, 0.0s] */
    if ((rtmGetTFinal(DataDriven_M)!=-1) &&
        !((rtmGetTFinal(DataDriven_M)-DataDriven_M->Timing.taskTime0) >
          DataDriven_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(DataDriven_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++DataDriven_M->Timing.clockTick0)) {
    ++DataDriven_M->Timing.clockTickH0;
  }

  DataDriven_M->Timing.taskTime0 = DataDriven_M->Timing.clockTick0 *
    DataDriven_M->Timing.stepSize0 + DataDriven_M->Timing.clockTickH0 *
    DataDriven_M->Timing.stepSize0 * 4294967296.0;
  rate_scheduler();
}

/* Model initialize function */
void DataDriven_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)DataDriven_M, 0,
                sizeof(RT_MODEL_DataDriven_T));
  rtmSetTFinal(DataDriven_M, 10.0);
  DataDriven_M->Timing.stepSize0 = 0.0032;

  /* block I/O */
  (void) memset(((void *) &DataDriven_B), 0,
                sizeof(B_DataDriven_T));

  /* states (dwork) */
  (void) memset((void *)&DataDriven_DW, 0,
                sizeof(DW_DataDriven_T));

  /* external inputs */
  DataDriven_U.Measurements = 0.0;

  /* external outputs */
  (void) memset((void *)&DataDriven_Y, 0,
                sizeof(ExtY_DataDriven_T));


  {
    int16_T i;

    /* SystemInitialize for MATLAB Function: '<S1>/Observer3' */
    DataDriven_DW.O_not_empty = false;

    /* '<S3>:1:7' yold=zeros(5,1); */
    for (i = 0; i < 5; i++) {
      DataDriven_DW.yold[i] = 0.0;
    }

    /* '<S3>:1:10' Step=0; */
    DataDriven_DW.Step = 0.0;

    /* End of SystemInitialize for MATLAB Function: '<S1>/Observer3' */

    /* SystemInitialize for MATLAB Function: '<S1>/Controller Variable Degree1' */
    /* '<S3>:1:13' z=zeros(3,1); */
    DataDriven_DW.K_not_empty = false;

    /* '<S2>:1:8' StateEstimate=zeros(3,5); */
    memset(&DataDriven_DW.StateEstimate[0], 0, 15U * sizeof(real_T));

    /* '<S2>:1:11' Uold=zeros(5,1); */
    /* '<S2>:1:14' Memory=zeros(5,1); */
    /* '<S2>:1:17' Step=0; */
    DataDriven_DW.Step_c = 0.0;

    /* '<S2>:1:20' BetaBar=zeros(5,1); */
    /* '<S2>:1:21' BetaBar(4)=1; */
    for (i = 0; i < 5; i++) {
      DataDriven_DW.Uold[i] = 0.0;
      DataDriven_DW.Memory[i] = 0.0;
      DataDriven_DW.BetaBar[i] = 0.0;
      DataDriven_DW.AlphaBar[i] = 0.0;
    }

    DataDriven_DW.BetaBar[3] = 1.0;

    /* '<S2>:1:24' AlphaBar=zeros(5,1); */
    /* '<S2>:1:25' AlphaBar(4)=0; */
    DataDriven_DW.AlphaBar[3] = 0.0;

    /* End of SystemInitialize for MATLAB Function: '<S1>/Controller Variable Degree1' */
  }
}

/* Model terminate function */
void DataDriven_terminate(void)
{
  /* (no terminate code required) */
}
