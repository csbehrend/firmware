/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: TVS.h
 *
 * Code generated for Simulink model 'TVS'.
 *
 * Model version                  : 2.446
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sat Apr 22 17:48:26 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_TVS_h_
#define RTW_HEADER_TVS_h_
#ifndef TVS_COMMON_INCLUDES_
#define TVS_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "bigM_v2_func.h"
#endif                                 /* TVS_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T PrevY[4];                     /* '<S2>/Tx_rate_limit' */
  real_T UnitDelay_DSTATE;             /* '<S5>/Unit Delay' */
  real_T UnitDelay_DSTATE_b;           /* '<S15>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S11>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<S11>/Unit Delay2' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: motorTempCurve
   * Referenced by: '<S3>/Temperature to Motor Current Limit Map'
   */
  real_T TemperaturetoMotorCurrentLimitM[11];

  /* Expression: motorControllerCurve(:,2)
   * Referenced by: '<S3>/Temperature to Motor Controller Current Limit Map'
   */
  real_T TemperaturetoMotorControllerCur[3];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by: '<S3>/Throttle Map '
   */
  real_T pooled10[11];

  /* Expression: Aeq_left
   * Referenced by: '<S5>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[261];

  /* Pooled Parameter (Expression: theta_sweep)
   * Referenced by:
   *   '<S5>/1-D Lookup Table'
   *   '<S5>/1-D Lookup Table1'
   */
  real_T pooled11[261];

  /* Expression: Aeq_right
   * Referenced by: '<S5>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[261];

  /* Expression: yaw_rate
   * Referenced by: '<S5>/2-D Lookup Table'
   */
  real_T uDLookupTable_tableData_p[405];

  /* Expression: steering_sweep
   * Referenced by: '<S5>/2-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[27];

  /* Expression: velocity_sweep
   * Referenced by: '<S5>/2-D Lookup Table'
   */
  real_T uDLookupTable_bp02Data[15];

  /* Computed Parameter: uDLookupTable_maxIndex
   * Referenced by: '<S5>/2-D Lookup Table'
   */
  uint32_T uDLookupTable_maxIndex[2];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T driver_input;                 /* '<Root>/driver_input' */
  real_T theta;                        /* '<Root>/steering_angle' */
  real_T omega[4];                     /* '<Root>/omega' */
  real_T pwr_lmt[2];                   /* '<Root>/current_limits' */
  real_T motor_V[4];                   /* '<Root>/motor_voltage' */
  real_T motor_I[4];                   /* '<Root>/motor_current' */
  real_T battery_V;                    /* '<Root>/battery_voltage' */
  real_T battery_I;                    /* '<Root>/battery_current' */
  real_T motor_T[4];                   /* '<Root>/motor_temp' */
  real_T mc_T[4];                      /* '<Root>/mc_temp' */
  real_T FZ[4];                        /* '<Root>/Fz' */
  real_T angvel_VNED[3];               /* '<Root>/angvel_VNED' */
  real_T vel_VNED[3];                  /* '<Root>/vel_VNED' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T ref_yaw;                      /* '<Root>/ref_yaw' */
  real_T beq;                          /* '<Root>/beq' */
  real_T obj_gain[4];                  /* '<Root>/obj_gain' */
  real_T lb[4];                        /* '<Root>/lb' */
  real_T ub[4];                        /* '<Root>/ub' */
  real_T windup;                       /* '<Root>/windup' */
  real_T Aeq[4];                       /* '<Root>/Aeq' */
  real_T b;                            /* '<Root>/b' */
  real_T max_b;                        /* '<Root>/max_b' */
  real_T Tx[4];                        /* '<Root>/Tx' */
  int32_T bigM_flag;                   /* '<Root>/bigM_flag' */
  real_T omega_m_TV[4];                /* '<Root>/omega_m_TV' */
  real_T max_ang_acc;                  /* '<Root>/max_ang_acc' */
  real_T min_ang_acc;                  /* '<Root>/min_ang_acc' */
  real_T motor_I[4];                   /* '<Root>/motor_I' */
  real_T motor_V[4];                   /* '<Root>/motor_V' */
  real_T I_limits[2];                  /* '<Root>/I_limits' */
  real_T batt_V;                       /* '<Root>/batt_V' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T ABS_MAX_TORQUE[4];            /* Variable: ABS_MAX_TORQUE
                                        * Referenced by:
                                        *   '<S2>/Tx_equal_offset'
                                        *   '<S2>/Tx_reverse_offset'
                                        *   '<S2>/lb_offset'
                                        *   '<S2>/ub_offset'
                                        *   '<S2>/optimization_offset_A'
                                        *   '<S2>/optimization_offset_Aeq'
                                        *   '<S2>/Tx_equal_gain'
                                        *   '<S3>/Constant3'
                                        *   '<S3>/Max Torque'
                                        */
  real_T BATTERY_CURRENT_MAX;          /* Variable: BATTERY_CURRENT_MAX
                                        * Referenced by: '<S3>/Current to Torque'
                                        */
  real_T CURRENT_BUFFER;               /* Variable: CURRENT_BUFFER
                                        * Referenced by: '<S3>/Max Torque Command Selector'
                                        */
  real_T I;                            /* Variable: I
                                        * Referenced by: '<S11>/Gain'
                                        */
  real_T MOTOR_ENABLE[4];              /* Variable: MOTOR_ENABLE
                                        * Referenced by:
                                        *   '<S2>/Tx_equal_gain'
                                        *   '<S3>/Gain4'
                                        *   '<S3>/Gain8'
                                        *   '<S5>/Constant2'
                                        *   '<S5>/Gain2'
                                        */
  real_T P;                            /* Variable: P
                                        * Referenced by: '<S5>/P'
                                        */
  real_T TVS_Intensity;                /* Variable: TVS_Intensity
                                        * Referenced by: '<S5>/Constant5'
                                        */
  real_T Torque_Down_Rate_Limit;       /* Variable: Torque_Down_Rate_Limit
                                        * Referenced by: '<S2>/Tx_rate_limit'
                                        */
  real_T Torque_Up_Rate_Limit;         /* Variable: Torque_Up_Rate_Limit
                                        * Referenced by: '<S2>/Tx_rate_limit'
                                        */
  real_T alpha_error;                  /* Variable: alpha_error
                                        * Referenced by:
                                        *   '<S5>/Gain'
                                        *   '<S5>/Gain1'
                                        */
  real_T deadband_angle;               /* Variable: deadband_angle
                                        * Referenced by: '<S5>/Constant1'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
  DW *dwork;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[3];
    } TaskCounters;
  } Timing;
};

/* Block parameters (default storage) */
extern P rtP;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void TVS_initialize(RT_MODEL *const rtM);
extern void TVS_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY);

#endif                                 /* RTW_HEADER_TVS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
