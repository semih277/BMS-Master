/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: BatterySOCEstimationV2.h
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

#ifndef BatterySOCEstimationV2_h_
#define BatterySOCEstimationV2_h_
#ifndef BatterySOCEstimationV2_COMMON_INCLUDES_
#define BatterySOCEstimationV2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                             /* BatterySOCEstimationV2_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>/SOC Estimator (Kalman Filter)1' */
typedef struct {
  real_T Probe[2];                     /* '<S6>/Probe' */
  real_T Delay_DSTATE[3];              /* '<S5>/Delay' */
  real_T UnitDelayP_DSTATE[9];         /* '<S3>/Unit Delay - P' */
  boolean_T icLoad;                    /* '<S5>/Delay' */
} DW_CoreSubsys;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_CoreSubsys CoreSubsys[1];     /* '<Root>/SOC Estimator (Kalman Filter)1' */
  real_T Assignment1[9];               /* '<S6>/Assignment1' */
  real_T Sum1_n[9];                    /* '<S7>/Sum1' */
  real_T dv[9];
  real_T Product3_e[3];                /* '<S4>/Product3' */
  real_T TmpSignalConversionAtProduc[3];
  real_T Sum_i[3];                     /* '<S4>/Sum' */
  real_T dv1[3];
  real_T uDLookupTableR2;              /* '<S6>/2-D Lookup Table R2' */
  real_T MathFunction1;                /* '<S6>/Math Function1' */
  real_T uDLookupTableR1;              /* '<S6>/2-D Lookup Table R1' */
  real_T MathFunction;                 /* '<S6>/Math Function' */
  real_T uDLookupTableOCV;             /* '<S6>/2-D Lookup Table OCV' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: eye(size(Q_2RC,1))
   * Referenced by: '<S4>/Constant2'
   */
  real_T Constant2_Value[9];

  /* Expression: [1 0  0;
     0 0 0;
     0 0 0]
   * Referenced by: '<S6>/Constant'
   */
  real_T Constant_Value[9];

  /* Expression: dV0_mat
   * Referenced by: '<S6>/2-D Lookup Table OCV1'
   */
  real_T uDLookupTableOCV1_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table OCV1'
   */
  real_T uDLookupTableOCV1_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table OCV1'
   */
  real_T uDLookupTableOCV1_bp02Data[2];

  /* Expression: R2_mat
   * Referenced by: '<S6>/2-D Lookup Table R2'
   */
  real_T uDLookupTableR2_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table R2'
   */
  real_T uDLookupTableR2_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table R2'
   */
  real_T uDLookupTableR2_bp02Data[2];

  /* Expression: C2_mat
   * Referenced by: '<S6>/2-D Lookup Table C2'
   */
  real_T uDLookupTableC2_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table C2'
   */
  real_T uDLookupTableC2_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table C2'
   */
  real_T uDLookupTableC2_bp02Data[2];

  /* Expression: R1_mat
   * Referenced by: '<S6>/2-D Lookup Table R1'
   */
  real_T uDLookupTableR1_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table R1'
   */
  real_T uDLookupTableR1_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table R1'
   */
  real_T uDLookupTableR1_bp02Data[2];

  /* Expression: C1_mat
   * Referenced by: '<S6>/2-D Lookup Table C1'
   */
  real_T uDLookupTableC1_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table C1'
   */
  real_T uDLookupTableC1_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table C1'
   */
  real_T uDLookupTableC1_bp02Data[2];

  /* Expression: P0_2RC
   * Referenced by: '<S3>/Unit Delay - P'
   */
  real_T UnitDelayP_InitialCondition[9];

  /* Expression: Q_2RC
   * Referenced by: '<S7>/Constant'
   */
  real_T Constant_Value_j[9];

  /* Expression: V0_mat
   * Referenced by: '<S6>/2-D Lookup Table OCV'
   */
  real_T uDLookupTableOCV_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table OCV'
   */
  real_T uDLookupTableOCV_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table OCV'
   */
  real_T uDLookupTableOCV_bp02Data[2];

  /* Expression: R0_mat
   * Referenced by: '<S6>/2-D Lookup Table R0'
   */
  real_T uDLookupTableR0_tableData[202];

  /* Expression: SOC_vec
   * Referenced by: '<S6>/2-D Lookup Table R0'
   */
  real_T uDLookupTableR0_bp01Data[101];

  /* Expression: T_vec
   * Referenced by: '<S6>/2-D Lookup Table R0'
   */
  real_T uDLookupTableR0_bp02Data[2];

  /* Pooled Parameter (Expression: flip(AE_ECM_params.OCV.z_rel_data))
   * Referenced by:
   *   '<Root>/25derece_SoC_OCV1'
   *   '<Root>/45derece_SoC_OCV1'
   */
  real_T pooled3[201];

  /* Expression: flip(AE_ECM_params.OCV.V_data(:,1))
   * Referenced by: '<Root>/45derece_SoC_OCV1'
   */
  real_T u5derece_SoC_OCV1_bp01Data[201];

  /* Expression: flip(AE_ECM_params.OCV.V_data(:,2))
   * Referenced by: '<Root>/25derece_SoC_OCV1'
   */
  real_T u5derece_SoC_OCV1_bp01Data_b[201];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S6>/2-D Lookup Table C1'
   *   '<S6>/2-D Lookup Table C2'
   *   '<S6>/2-D Lookup Table OCV'
   *   '<S6>/2-D Lookup Table OCV1'
   *   '<S6>/2-D Lookup Table R0'
   *   '<S6>/2-D Lookup Table R1'
   *   '<S6>/2-D Lookup Table R2'
   */
  uint32_T pooled4[2];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T current;                      /* '<Root>/current' */
  real_T TerminalVoltage;              /* '<Root>/TerminalVoltage' */
  real_T Temperature;                  /* '<Root>/Temperature' */
  real_T OCV;                          /* '<Root>/OCV' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T SoC;                          /* '<Root>/SoC' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void BatterySOCEstimationV2_initialize(void);
extern void BatterySOCEstimationV2_step(void);
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S9>/Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/Data Type Duplicate' : Unused code path elimination
 * Block '<S11>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S14>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<Root>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<S8>/Conversion' : Eliminate redundant data type conversion
 * Block '<S9>/Conversion' : Eliminate redundant data type conversion
 * Block '<S10>/Conversion' : Eliminate redundant data type conversion
 * Block '<S11>/Conversion' : Eliminate redundant data type conversion
 * Block '<S12>/Conversion' : Eliminate redundant data type conversion
 * Block '<S13>/Conversion' : Eliminate redundant data type conversion
 * Block '<S14>/Conversion' : Eliminate redundant data type conversion
 * Block '<S15>/Conversion' : Eliminate redundant data type conversion
 * Block '<S3>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S3>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S3>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<S3>/Rate Transition3' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BatterySOCEstimationV2'
 * '<S1>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1'
 * '<S2>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter'
 * '<S3>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC'
 * '<S4>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Correction'
 * '<S5>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Delay - X'
 * '<S6>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Jacobian'
 * '<S7>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Prediction'
 * '<S8>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited'
 * '<S9>'   : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited1'
 * '<S10>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited2'
 * '<S11>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Delay - X/Data Type Conversion Inherited'
 * '<S12>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited'
 * '<S13>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited1'
 * '<S14>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited2'
 * '<S15>'  : 'BatterySOCEstimationV2/SOC Estimator (Kalman Filter)1/Kalman Filter/EKF 2RC/Prediction/Data Type Conversion Inherited'
 */
#endif                                 /* BatterySOCEstimationV2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
