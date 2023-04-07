#include "SFS.h" 

// clamping
#define MIN_MAG 200.0
#define MAX_MAG -200.0

#define MIN_GYRO -2.0
#define MAX_GYRO 2.0

#define MIN_ACC -25.0
#define MAX_ACC 25.0

#define MIN_POS -300.0
#define MAX_POS 300.0

#define MIN_VEL -30.0
#define MAX_VEL 30.0

void SFS_pp(ExtU* rtU);