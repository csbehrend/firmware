#include "TVS.h" 

// Calibration
#define DRIVER_INPUT_CALIBRATION 1.0 / 4095.0
#define STEERING_ANGLE_CALIBRATION 0.1
#define OMEGA_CALIBRATION 1.0 / (360.0 * 0.2286)
#define CL_CALIBRATION 1.0
#define MOTOR_V_CALIBRATION 1.0
#define MOTOR_I_CALIBRATION 1.0
#define BATTERY_V_CALIBRATION 0.1
#define BATTERY_I_CALIBRATION 1.0
#define MOTOR_T_CALIBRATION 1.0
#define MC_T_CALIBRATION 1.0
#define FZ_CALIBRATION 1.0
#define GYRO_CALIBRATION 1.0
#define VEL_CALIBRATION 1.0

//#define ANGLE_CALIBRATION 1.0
//#define ROTATION_CALIBRATION 1.0
//#define ACCELERATION_CALIBRATION 1.0
//#define CURRENT_CALIBRATION 1.0
//#define SHOCK_CALIBRATION 1.0 / 1000.0

// Clamping
#define MIN_THROTTLE 0.0
#define MAX_THROTTLE 1.0

#define MIN_STEERING -130.0
#define MAX_STEERING 130.0

#define MIN_OMEGA 0.0
#define MAX_OMEGA 150.0

#define MAX_DCL 140.0
#define MAX_CCL 0.0
#define MIN_CL 0.0

#define MIN_MOTOR_V 90.0
#define MAX_MOTOR_V 340.0

#define MIN_MOTOR_I 0.0
#define MAX_MOTOR_I 70.0

#define MIN_BATTERY_V 200.0
#define MAX_BATTERY_V 340.0

#define MIN_BATTERY_I 0.0
#define MAX_BATTERY_I 140.0

#define MIN_MOTOR_T 0.0
#define MAX_MOTOR_T 100.0

#define MIN_MC_T 0.0
#define MAX_MC_T 100.0

#define MIN_FZ 200.0
#define MAX_FZ 1200.0

#define MIN_ANG_VEL -5.0
#define MAX_ANG_VEL 5.0

#define MIN_VEL 0.0
#define MAX_VEL 30.0

//#define MIN_ACCEL -30.0
//#define MAX_ACCEL 30.0

//#define MIN_SHOCK_D 0.1
//#define MAX_SHOCK_D 0.25

//#define MIN_SHOCK_V -0.25
//#define MAX_SHOCK_V 0.25

//#define MIN_ANG -360.0
//#define MAX_ANG 360.0

void TV_pp(ExtU* rtU);
