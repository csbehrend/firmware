/**
 * @file TorqueVector.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _TORQUE_VECTOR_H_
#define _TORQUE_VECTOR_H_

typedef struct
{
    float half_track_width[2];      /* Half track width [m][1x2] */
    float wheelbase[2];             /* Wheelbase [m][1x2] */
    float RE;                       /* Effective Radius of tire [m][1x1] */
    float mu_factor;                /* Coefficient of Friction Modifier [1x1] */
    float k_limit;                  /* Max Combined Slip before problems happen [1x1]*/
    float tire_mu;                  /* Maximum Friction Coefficient [Unitless][1x1] */
    float gear_ratios[4];           /* Gear ratios [Unitless][1x4] */
    float gearbox_efficiency[4];    /* Gearbox Power Transmission Efficiency [Unitless][1x4] */
    float r;                        /* Tire Radius [m][1x1] */
    float K_u;                      /* Understeer Gradient [rad/m/s^2][1x1] */
    float c_factor;                 /* Max Yaw Equation Contributing Factor [Unitless][1x1] */
    float brakeforce_max[4];        /* Max Force Brakes Can Exert On Each Tire Axle [N][1x4] */
    float brakepad_mu;              /* Brakepad Friction Coefficient[Unitless][1x1] */
    float disk_diameter;            /* Diameter of brake disk [m][1x1] */
    float mech_brake_thresh;        /* Value Of driver_input Where Mechanical Braking Begins [Unitless][1x1] */
    float disk_diameter;            /* Diameter of brake disk [m][1x1] */
    float yaw_factor; // IMPORTANT  /* Coefficent to adjust desired yaw magnitude [1x1] */
    float J_z;                      /* Polar Moment Of Inertia [kg m^2][1x1] */
    float motor_efficiency[4];      /* Motor efficiency [Unitless][1x4] */
    float motor_limit_torque[2];    /* Minimum & Max Torque Request From Motor [Nm][1x2] */
    float min_speed_regen;          /* Min Motor Shaft Speed For Regen Braking [rad/s][1x1] */
} VehicleConfiguration_t;

void TorqueVector_main();

#endif