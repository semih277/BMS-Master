/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: BatterySOCEstimationV2.c
 *
 * Code generated for Simulink model 'BatterySOCEstimationV2'.
 *
 * Model version                  : 13.9
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Tue Jul  9 16:54:24 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "BatterySOCEstimationV2.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;
static real_T look2_binlx(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride);
static real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
static real_T look2_binlx(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride)
{
  real_T fractions[2];
  real_T frac;
  real_T yL_0d0;
  real_T yL_0d1;
  uint32_T bpIndices[2];
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 2-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex[0U]]) {
    /* Binary Search */
    bpIdx = maxIndex[0U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[0U];
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U] - 1U;
    frac = (u0 - bp0[maxIndex[0U] - 1U]) / (bp0[maxIndex[0U]] - bp0[maxIndex[0U]
      - 1U]);
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = (u1 - bp1[0U]) / (bp1[1U] - bp1[0U]);
  } else if (u1 < bp1[maxIndex[1U]]) {
    /* Binary Search */
    bpIdx = maxIndex[1U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[1U];
    while (iRght - iLeft > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U] - 1U;
    frac = (u1 - bp1[maxIndex[1U] - 1U]) / (bp1[maxIndex[1U]] - bp1[maxIndex[1U]
      - 1U]);
  }

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  bpIdx = iLeft * stride + bpIndices[0U];
  yL_0d0 = table[bpIdx];
  yL_0d0 += (table[bpIdx + 1U] - yL_0d0) * fractions[0U];
  bpIdx += stride;
  yL_0d1 = table[bpIdx];
  return (((table[bpIdx + 1U] - yL_0d1) * fractions[0U] + yL_0d1) - yL_0d0) *
    frac + yL_0d0;
}

static real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/* Model step function */
void BatterySOCEstimationV2_step(void)
{
  real_T Switch;
  real_T rtb_Assignment1_0;
  real_T rtb_Assignment1_1;
  real_T rtb_Sum1_n_0;
  real_T rtb_TmpSignalConversionAtProd_0;
  int32_T i;
  int32_T i_0;
  int32_T rtb_Sum1_n_tmp;

  /* Switch: '<Root>/Switch' incorporates:
   *  Inport: '<Root>/Temperature'
   */
  if (rtU.Temperature > 35.0) {
    /* Switch: '<Root>/Switch' incorporates:
     *  Inport: '<Root>/OCV'
     *  Lookup_n-D: '<Root>/45derece_SoC_OCV1'
     */
    Switch = look1_binlx(rtU.OCV, rtConstP.u5derece_SoC_OCV1_bp01Data,
                         rtConstP.pooled3, 200U);
  } else {
    /* Switch: '<Root>/Switch' incorporates:
     *  Inport: '<Root>/OCV'
     *  Lookup_n-D: '<Root>/25derece_SoC_OCV1'
     */
    Switch = look1_binlx(rtU.OCV, rtConstP.u5derece_SoC_OCV1_bp01Data_b,
                         rtConstP.pooled3, 200U);
  }

  /* End of Switch: '<Root>/Switch' */

  /* Outputs for Iterator SubSystem: '<Root>/SOC Estimator (Kalman Filter)1' incorporates:
   *  ForEach: '<S1>/For Each'
   */
  /* Delay: '<S5>/Delay' incorporates:
   *  Constant: '<S5>/Constant1'
   *  ForEachSliceSelector generated from: '<S1>/InitialSOC'
   */
  if (rtDW.CoreSubsys[0].icLoad) {
    rtDW.CoreSubsys[0].Delay_DSTATE[0] = Switch;
    rtDW.CoreSubsys[0].Delay_DSTATE[1] = 0.0;
    rtDW.CoreSubsys[0].Delay_DSTATE[2] = 0.0;
  }

  /* SignalConversion generated from: '<S6>/Transpose1' incorporates:
   *  Constant: '<S6>/Constant4'
   *  Delay: '<S5>/Delay'
   *  Inport: '<Root>/Temperature'
   *  Lookup_n-D: '<S6>/2-D Lookup Table OCV1'
   */
  rtDW.Product3_e[0] = look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0],
    rtU.Temperature, rtConstP.uDLookupTableOCV1_bp01Data,
    rtConstP.uDLookupTableOCV1_bp02Data, rtConstP.uDLookupTableOCV1_tableData,
    rtConstP.pooled4, 101U);
  rtDW.Product3_e[1] = -1.0;
  rtDW.Product3_e[2] = -1.0;

  /* Lookup_n-D: '<S6>/2-D Lookup Table R2' incorporates:
   *  Delay: '<S5>/Delay'
   *  Inport: '<Root>/Temperature'
   */
  rtDW.uDLookupTableR2 = look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0],
    rtU.Temperature, rtConstP.uDLookupTableR2_bp01Data,
    rtConstP.uDLookupTableR2_bp02Data, rtConstP.uDLookupTableR2_tableData,
    rtConstP.pooled4, 101U);

  /* Math: '<S6>/Math Function1' incorporates:
   *  Delay: '<S5>/Delay'
   *  Gain: '<S6>/Gain1'
   *  Inport: '<Root>/Temperature'
   *  Lookup_n-D: '<S6>/2-D Lookup Table C2'
   *  Product: '<S6>/Product1'
   *  Product: '<S6>/Product3'
   *
   * About '<S6>/Math Function1':
   *  Operator: exp
   */
  rtDW.MathFunction1 = exp(-rtDW.CoreSubsys[0].Probe[0] / (rtDW.uDLookupTableR2 *
    look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0], rtU.Temperature,
                rtConstP.uDLookupTableC2_bp01Data,
                rtConstP.uDLookupTableC2_bp02Data,
                rtConstP.uDLookupTableC2_tableData, rtConstP.pooled4, 101U)));

  /* Lookup_n-D: '<S6>/2-D Lookup Table R1' incorporates:
   *  Delay: '<S5>/Delay'
   *  Inport: '<Root>/Temperature'
   */
  rtDW.uDLookupTableR1 = look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0],
    rtU.Temperature, rtConstP.uDLookupTableR1_bp01Data,
    rtConstP.uDLookupTableR1_bp02Data, rtConstP.uDLookupTableR1_tableData,
    rtConstP.pooled4, 101U);

  /* Math: '<S6>/Math Function' incorporates:
   *  Delay: '<S5>/Delay'
   *  Gain: '<S6>/Gain1'
   *  Inport: '<Root>/Temperature'
   *  Lookup_n-D: '<S6>/2-D Lookup Table C1'
   *  Product: '<S6>/Product'
   *  Product: '<S6>/Product2'
   *
   * About '<S6>/Math Function':
   *  Operator: exp
   */
  rtDW.MathFunction = exp(-rtDW.CoreSubsys[0].Probe[0] / (rtDW.uDLookupTableR1 *
    look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0], rtU.Temperature,
                rtConstP.uDLookupTableC1_bp01Data,
                rtConstP.uDLookupTableC1_bp02Data,
                rtConstP.uDLookupTableC1_tableData, rtConstP.pooled4, 101U)));

  /* SignalConversion generated from: '<S6>/Assignment' incorporates:
   *  Assignment: '<S6>/Assignment1'
   *  Constant: '<S6>/Constant'
   */
  memcpy(&rtDW.Assignment1[0], &rtConstP.Constant_Value[0], 9U * sizeof(real_T));

  /* Assignment: '<S6>/Assignment' incorporates:
   *  Assignment: '<S6>/Assignment1'
   */
  rtDW.Assignment1[4] = rtDW.MathFunction;

  /* Assignment: '<S6>/Assignment1' */
  rtDW.Assignment1[8] = rtDW.MathFunction1;

  /* Product: '<S7>/Product2' incorporates:
   *  Assignment: '<S6>/Assignment1'
   *  Math: '<S7>/Transpose'
   *  UnitDelay: '<S3>/Unit Delay - P'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (i = 0; i < 3; i++) {
      rtDW.dv[i_0 + 3 * i] = (rtDW.CoreSubsys[0].UnitDelayP_DSTATE[i_0 + 3] *
        rtDW.Assignment1[i + 3] + rtDW.CoreSubsys[0].UnitDelayP_DSTATE[i_0] *
        rtDW.Assignment1[i]) + rtDW.CoreSubsys[0].UnitDelayP_DSTATE[i_0 + 6] *
        rtDW.Assignment1[i + 6];
    }
  }

  /* Product: '<S4>/Product2' */
  rtDW.uDLookupTableOCV = 0.0;
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Sum: '<S7>/Sum1' incorporates:
     *  Assignment: '<S6>/Assignment1'
     *  Product: '<S7>/Product2'
     */
    Switch = rtDW.Assignment1[i_0 + 3];
    rtb_Assignment1_0 = rtDW.Assignment1[i_0];
    rtb_Assignment1_1 = rtDW.Assignment1[i_0 + 6];

    /* Product: '<S4>/Product2' */
    rtb_TmpSignalConversionAtProd_0 = 0.0;
    for (i = 0; i < 3; i++) {
      /* Sum: '<S7>/Sum1' incorporates:
       *  Constant: '<S7>/Constant'
       *  Product: '<S7>/Product2'
       */
      rtb_Sum1_n_tmp = 3 * i + i_0;
      rtb_Sum1_n_0 = ((rtDW.dv[3 * i + 1] * Switch + rtDW.dv[3 * i] *
                       rtb_Assignment1_0) + rtDW.dv[3 * i + 2] *
                      rtb_Assignment1_1) +
        rtConstP.Constant_Value_j[rtb_Sum1_n_tmp];
      rtDW.Sum1_n[rtb_Sum1_n_tmp] = rtb_Sum1_n_0;

      /* Product: '<S4>/Product2' incorporates:
       *  Math: '<S4>/Transpose'
       *  Product: '<S4>/Product'
       *  Product: '<S4>/Product3'
       */
      rtb_TmpSignalConversionAtProd_0 += rtb_Sum1_n_0 * rtDW.Product3_e[i];
    }

    /* Product: '<S4>/Product2' incorporates:
     *  Product: '<S4>/Product3'
     */
    rtDW.TmpSignalConversionAtProduc[i_0] = rtb_TmpSignalConversionAtProd_0;
    rtDW.uDLookupTableOCV += rtDW.Product3_e[i_0] *
      rtb_TmpSignalConversionAtProd_0;
  }

  /* Product: '<S4>/Divide' incorporates:
   *  Constant: '<S4>/Constant'
   *  Constant: '<S4>/Constant1'
   *  Product: '<S4>/Product2'
   *  Sum: '<S4>/Sum2'
   */
  rtDW.uDLookupTableOCV = 1.0 / (rtDW.uDLookupTableOCV + 0.7);

  /* Product: '<S4>/Product1' incorporates:
   *  Product: '<S4>/Product'
   *  SignalConversion generated from: '<S7>/Product1'
   */
  rtDW.TmpSignalConversionAtProduc[0] *= rtDW.uDLookupTableOCV;
  rtDW.TmpSignalConversionAtProduc[1] *= rtDW.uDLookupTableOCV;
  rtDW.TmpSignalConversionAtProduc[2] *= rtDW.uDLookupTableOCV;

  /* Sum: '<S3>/Sum' incorporates:
   *  Delay: '<S5>/Delay'
   *  Gain: '<S1>/Gain'
   *  Inport: '<Root>/Temperature'
   *  Inport: '<Root>/TerminalVoltage'
   *  Inport: '<Root>/current'
   *  Lookup_n-D: '<S6>/2-D Lookup Table OCV'
   *  Lookup_n-D: '<S6>/2-D Lookup Table R0'
   *  Product: '<S6>/Product6'
   *  Sum: '<S6>/Sum of Elements'
   *  Sum: '<S6>/Sum2'
   */
  rtDW.uDLookupTableOCV = rtU.TerminalVoltage - ((look2_binlx(rtDW.CoreSubsys[0]
    .Delay_DSTATE[0], rtU.Temperature, rtConstP.uDLookupTableOCV_bp01Data,
    rtConstP.uDLookupTableOCV_bp02Data, rtConstP.uDLookupTableOCV_tableData,
    rtConstP.pooled4, 101U) - look2_binlx(rtDW.CoreSubsys[0].Delay_DSTATE[0],
    rtU.Temperature, rtConstP.uDLookupTableR0_bp01Data,
    rtConstP.uDLookupTableR0_bp02Data, rtConstP.uDLookupTableR0_tableData,
    rtConstP.pooled4, 101U) * -rtU.current) - (rtDW.CoreSubsys[0].Delay_DSTATE[1]
    + rtDW.CoreSubsys[0].Delay_DSTATE[2]));

  /* SignalConversion generated from: '<S7>/Product1' incorporates:
   *  Constant: '<S6>/Constant1'
   *  Gain: '<S1>/Gain'
   *  Gain: '<S6>/Gain'
   *  Gain: '<S6>/Gain1'
   *  Inport: '<Root>/current'
   *  Product: '<S6>/Product4'
   *  Product: '<S6>/Product5'
   *  Product: '<S7>/Product1'
   *  Sum: '<S6>/Sum'
   *  Sum: '<S6>/Sum1'
   */
  rtDW.dv1[0] = 6.6137566137566142E-5 * -rtDW.CoreSubsys[0].Probe[0] *
    -rtU.current;
  rtDW.dv1[1] = (1.0 - rtDW.MathFunction) * rtDW.uDLookupTableR1 * -rtU.current;
  rtDW.dv1[2] = (1.0 - rtDW.MathFunction1) * rtDW.uDLookupTableR2 * -rtU.current;
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Sum: '<S4>/Sum' incorporates:
     *  Assignment: '<S6>/Assignment1'
     *  Delay: '<S5>/Delay'
     *  Product: '<S4>/Product3'
     *  Product: '<S7>/Product'
     *  SignalConversion generated from: '<S7>/Product1'
     *  Sum: '<S7>/Sum'
     */
    rtDW.Sum_i[i_0] = (((rtDW.Assignment1[i_0 + 3] * rtDW.CoreSubsys[0].
                         Delay_DSTATE[1] + rtDW.Assignment1[i_0] *
                         rtDW.CoreSubsys[0].Delay_DSTATE[0]) +
                        rtDW.Assignment1[i_0 + 6] * rtDW.CoreSubsys[0].
                        Delay_DSTATE[2]) + rtDW.dv1[i_0]) +
      rtDW.TmpSignalConversionAtProduc[i_0] * rtDW.uDLookupTableOCV;
  }

  /* Update for Delay: '<S5>/Delay' */
  rtDW.CoreSubsys[0].icLoad = false;

  /* Product: '<S4>/Product4' incorporates:
   *  SignalConversion generated from: '<S7>/Product1'
   */
  rtb_TmpSignalConversionAtProd_0 = rtDW.TmpSignalConversionAtProduc[0];
  rtDW.uDLookupTableR2 = rtDW.TmpSignalConversionAtProduc[1];
  rtDW.MathFunction1 = rtDW.TmpSignalConversionAtProduc[2];
  for (i = 0; i < 3; i++) {
    /* Update for Delay: '<S5>/Delay' */
    rtDW.CoreSubsys[0].Delay_DSTATE[i] = rtDW.Sum_i[i];

    /* Product: '<S4>/Product4' incorporates:
     *  Product: '<S4>/Product3'
     */
    rtDW.uDLookupTableOCV = rtDW.Product3_e[i];

    /* Sum: '<S4>/Sum3' incorporates:
     *  Constant: '<S4>/Constant2'
     *  Product: '<S4>/Product3'
     *  Product: '<S4>/Product4'
     *  SignalConversion generated from: '<S7>/Product1'
     */
    rtDW.dv[3 * i] = rtConstP.Constant2_Value[3 * i] -
      rtb_TmpSignalConversionAtProd_0 * rtDW.uDLookupTableOCV;
    i_0 = 3 * i + 1;
    rtDW.dv[i_0] = rtConstP.Constant2_Value[i_0] - rtDW.uDLookupTableR2 *
      rtDW.uDLookupTableOCV;
    i_0 = 3 * i + 2;
    rtDW.dv[i_0] = rtConstP.Constant2_Value[i_0] - rtDW.MathFunction1 *
      rtDW.uDLookupTableOCV;
  }

  /* Update for UnitDelay: '<S3>/Unit Delay - P' */
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Product: '<S4>/Product5' */
    rtDW.uDLookupTableR2 = rtDW.dv[i_0];
    rtDW.MathFunction1 = rtDW.dv[i_0 + 3];
    rtDW.uDLookupTableR1 = rtDW.dv[i_0 + 6];
    for (i = 0; i < 3; i++) {
      /* Product: '<S4>/Product5' incorporates:
       *  Sum: '<S7>/Sum1'
       */
      rtDW.CoreSubsys[0].UnitDelayP_DSTATE[i_0 + 3 * i] = (rtDW.Sum1_n[3 * i + 1]
        * rtDW.MathFunction1 + rtDW.Sum1_n[3 * i] * rtDW.uDLookupTableR2) +
        rtDW.Sum1_n[3 * i + 2] * rtDW.uDLookupTableR1;
    }
  }

  /* End of Update for UnitDelay: '<S3>/Unit Delay - P' */

  /* ForEachSliceAssignment generated from: '<S1>/SOC' */
  rtDW.uDLookupTableR2 = rtDW.Sum_i[0];

  /* End of Outputs for SubSystem: '<Root>/SOC Estimator (Kalman Filter)1' */

  /* Outport: '<Root>/SoC' */
  rtY.SoC = rtDW.uDLookupTableR2;
}

/* Model initialize function */
void BatterySOCEstimationV2_initialize(void)
{
  /* SystemInitialize for Iterator SubSystem: '<Root>/SOC Estimator (Kalman Filter)1' */
  /* Start for Probe: '<S6>/Probe' */
  rtDW.CoreSubsys[0].Probe[0] = 0.05;
  rtDW.CoreSubsys[0].Probe[1] = 0.0;

  /* InitializeConditions for Delay: '<S5>/Delay' */
  rtDW.CoreSubsys[0].icLoad = true;

  /* InitializeConditions for UnitDelay: '<S3>/Unit Delay - P' */
  memcpy(&rtDW.CoreSubsys[0].UnitDelayP_DSTATE[0],
         &rtConstP.UnitDelayP_InitialCondition[0], 9U * sizeof(real_T));

  /* End of SystemInitialize for SubSystem: '<Root>/SOC Estimator (Kalman Filter)1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
