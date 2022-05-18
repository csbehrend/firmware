#include "shockpot.h"
#include "can_parse.h"
#include "common/phal_L4/dma/dma.h"
#include "force.h"

extern q_handle_t q_tx_can;

static int shockPots [2][N_SAMPLE] = {0};         // 0 - left, 1 - right
// TODO: convert to using the data struct
volatile raw_shock_pots_t raw_shock_pots;
int start = 0;

// float n_rear_left;
// float n_rear_right;
// float n_front_left;
// float n_front_right;

float pot_speed_r;
float pot_speed_l;

float n_rear_left;
float n_rear_right;
float n_front_left;
float n_front_right;

void shockpotInit()
{

}

void shockpot200Hz()
{
    shockPots[0][start] = raw_shock_pots.pot_left;
    shockPots[1][start] = raw_shock_pots.pot_right;
    
    normal_force(&n_rear_left, &n_rear_right, shockPots[0], shockPots[1], start);
    // n_rear (shockPots[0], shockPots[1], &n_rear_left, &n_rear_right, start);
    start = (start + N_SAMPLE - 1) % N_SAMPLE;
    //  SEND_FRONT_WHEEL_DATA(q_tx_can, 0, 0, n_rear_left, n_rear_right);
}

