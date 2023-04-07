#include "SFS_pp.h"
#include "SFS.h"
#include "can_parse.h"
#include "common_defs.h"

void SFS_pp(ExtU* rtU)
{
    rtU->mag[0] = CLAMP(0, MIN_MAG, MAX_MAG);
    rtU->mag[1] = CLAMP(0, MIN_MAG, MAX_MAG);
    rtU->mag[2] = CLAMP(0, MIN_MAG, MAX_MAG);

    rtU->gyro[0] = CLAMP(0, MIN_GYRO, MAX_GYRO);
    rtU->gyro[1] = CLAMP(0, MIN_GYRO, MAX_GYRO);
    rtU->gyro[2] = CLAMP(0, MIN_GYRO, MAX_GYRO);

    rtU->acc[0] = CLAMP(0, MIN_ACC, MAX_ACC);
    rtU->acc[1] = CLAMP(0, MIN_ACC, MAX_ACC);
    rtU->acc[2] = CLAMP(0, MIN_ACC, MAX_ACC);

    if (0)
    {
        rtU->pos[0] = CLAMP(0, MIN_POS, MAX_POS);
        rtU->pos[1] = CLAMP(0, MIN_POS, MAX_POS);
        rtU->pos[2] = CLAMP(0, MIN_POS, MAX_POS);

        rtU->vel[0] = CLAMP(0, MIN_VEL, MAX_VEL);
        rtU->vel[1] = CLAMP(0, MIN_VEL, MAX_VEL);
        rtU->vel[2] = CLAMP(0, MIN_VEL, MAX_VEL);
    }
    else
    {
        rtU->pos[0] = CLAMP(0, MIN_POS, MAX_POS);
        rtU->pos[1] = CLAMP(0, MIN_POS, MAX_POS);
        rtU->pos[2] = CLAMP(0, MIN_POS, MAX_POS);

        rtU->vel[0] = CLAMP(0, MIN_VEL, MAX_VEL);
        rtU->vel[1] = CLAMP(0, MIN_VEL, MAX_VEL);
        rtU->vel[2] = CLAMP(0, MIN_VEL, MAX_VEL);
    }
}
