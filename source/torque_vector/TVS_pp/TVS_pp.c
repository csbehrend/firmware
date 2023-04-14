#include "TVS_pp.h"
#include "TVS.h"
#include "can_parse.h"
#include "common_defs.h"

void TV_pp(ExtU* rtU)
{
rtU->driver_input = CLAMP(can_data.raw_throttle_brake.throttle * DRIVER_INPUT_CALIBRATION, MIN_THROTTLE, MAX_THROTTLE);

rtU->theta = CLAMP(can_data.LWS_Standard.LWS_ANGLE * STEERING_ANGLE_CALIBRATION, MIN_STEERING, MAX_STEERING);

rtU->omega[0] = CLAMP(0.0 * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[1] = CLAMP(0.0 * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[2] = CLAMP(can_data.rear_wheel_data.left_speed * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[3] = CLAMP(can_data.rear_wheel_data.right_speed * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);

rtU->pwr_lmt[0] = CLAMP(can_data.orion_info.pack_dcl * CL_CALIBRATION, MIN_CL, MAX_DCL);
rtU->pwr_lmt[1] = CLAMP(can_data.orion_info.pack_ccl * CL_CALIBRATION, MIN_CL, MAX_CCL);

rtU->motor_V[0] = CLAMP(0 * MOTOR_V_CALIBRATION, MIN_MOTOR_V, MAX_MOTOR_V);
rtU->motor_V[0] = CLAMP(0 * MOTOR_V_CALIBRATION, MIN_MOTOR_V, MAX_MOTOR_V);
rtU->motor_V[0] = CLAMP(can_data.rear_motor_currents_temps.right_voltage * MOTOR_V_CALIBRATION, MIN_MOTOR_V, MAX_MOTOR_V);
rtU->motor_V[0] = CLAMP(can_data.rear_motor_currents_temps.right_voltage * MOTOR_V_CALIBRATION, MIN_MOTOR_V, MAX_MOTOR_V);

rtU->motor_I[0] = CLAMP(0 * MOTOR_I_CALIBRATION, MIN_MOTOR_I, MAX_MOTOR_I);
rtU->motor_I[0] = CLAMP(0 * MOTOR_I_CALIBRATION, MIN_MOTOR_I, MAX_MOTOR_I);
rtU->motor_I[0] = CLAMP(can_data.rear_motor_currents_temps.left_current * MOTOR_I_CALIBRATION, MIN_MOTOR_I, MAX_MOTOR_I);
rtU->motor_I[0] = CLAMP(can_data.rear_motor_currents_temps.right_current * MOTOR_I_CALIBRATION, MIN_MOTOR_I, MAX_MOTOR_I);

rtU->battery_V = CLAMP(can_data.orion_currents_volts.pack_voltage * BATTERY_V_CALIBRATION, MIN_BATTERY_V, MAX_BATTERY_V);
rtU->battery_I = CLAMP(can_data.orion_currents_volts.pack_current * BATTERY_I_CALIBRATION, MIN_BATTERY_I, MAX_BATTERY_I);

rtU->motor_T[0] = CLAMP(55.0 * MOTOR_T_CALIBRATION, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_T[1] = CLAMP(55.0 * MOTOR_T_CALIBRATION, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_T[2] = CLAMP(can_data.rear_motor_currents_temps.left_temp * MOTOR_T_CALIBRATION, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_T[3] = CLAMP(can_data.rear_motor_currents_temps.right_temp * MOTOR_T_CALIBRATION, MIN_MOTOR_T, MAX_MOTOR_T);

rtU->mc_T[0] = CLAMP(55.0 * MC_T_CALIBRATION, MIN_MC_T, MAX_MC_T);
rtU->mc_T[1] = CLAMP(55.0 * MC_T_CALIBRATION, MIN_MC_T, MAX_MC_T);
rtU->mc_T[2] = CLAMP(can_data.rear_controller_temps.left_temp * MC_T_CALIBRATION, MIN_MC_T, MAX_MC_T);
rtU->mc_T[3] = CLAMP(can_data.rear_controller_temps.right_temp * MC_T_CALIBRATION, MIN_MC_T, MAX_MC_T);

rtU->FZ[0] = CLAMP(1200.0 * FZ_CALIBRATION, MIN_FZ, MAX_FZ);
rtU->FZ[1] = CLAMP(1200.0 * FZ_CALIBRATION, MIN_FZ, MAX_FZ);
rtU->FZ[2] = CLAMP(1200.0 * FZ_CALIBRATION, MIN_FZ, MAX_FZ);
rtU->FZ[3] = CLAMP(1200.0 * FZ_CALIBRATION, MIN_FZ, MAX_FZ);

//rtU->angvel_VNED[0] = CLAMP(can_data.gyro_data.gx * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
//rtU->angvel_VNED[1] = CLAMP(can_data.gyro_data.gy * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
//rtU->angvel_VNED[2] = CLAMP(can_data.gyro_data.gz * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);

rtU->angvel_VNED[0] = CLAMP(0.0 * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
rtU->angvel_VNED[1] = CLAMP(0.0 * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
rtU->angvel_VNED[2] = CLAMP(0.0 * GYRO_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);

//rtU->vel_VNED[0] = CLAMP((rtU->omega[2] + rtU->omega[3]) * 0.222 / 2, MIN_VEL, MAX_VEL);
rtU->vel_VNED[0] = CLAMP(0.0 * VEL_CALIBRATION, MIN_VEL, MAX_VEL);
rtU->vel_VNED[1] = CLAMP(0.0 * VEL_CALIBRATION, MIN_VEL, MAX_VEL);
rtU->vel_VNED[2] = CLAMP(0.0 * VEL_CALIBRATION, MIN_VEL, MAX_VEL);

//rtU->ang[0] = CLAMP(can_data.angle_data.pitch * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[1] = CLAMP(can_data.angle_data.roll * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[2] = CLAMP(can_data.angle_data.yaw * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);

//rtU->accel[0] = CLAMP(can_data.accel_data.ax * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);
//rtU->accel[1] = CLAMP(can_data.accel_data.ay * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);
//rtU->accel[2] = CLAMP(can_data.accel_data.az * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);

//rtU->shock_displacement[0] = CLAMP(0.0, MIN_SHOCK_D, MAX_SHOCK_D);
//rtU->shock_displacement[1] = CLAMP(0.0, MIN_SHOCK_D, MAX_SHOCK_D);
//rtU->shock_displacement[2] = CLAMP(can_data.rear_wheel_data.left_normal * SHOCK_CALIBRATION, MIN_SHOCK_D, MAX_SHOCK_D);
//rtU->shock_displacement[3] = CLAMP(can_data.rear_wheel_data.right_normal * SHOCK_CALIBRATION, MIN_SHOCK_D, MAX_SHOCK_D);

//rtU->shock_velocity[0] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
//rtU->shock_velocity[1] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
//rtU->shock_velocity[2] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
//rtU->shock_velocity[3] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);

//rtU->ang[0] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[1] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[2] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);

}