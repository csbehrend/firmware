#include "shockpot.h"
#include "can_parse.h"
#include "common/phal_L4/dma/dma.h"
#include "force.h"
#include "main.h"

static uint16_t shockPots [2][N_REAR] = {0};         // 0 - left, 1 - right
// TODO: convert to using the data struct
int start = 0;

// float n_rear_left;
// float n_rear_right;
// float n_front_left;
// float n_front_right;

float pot_speed_r;
float pot_speed_l;

float n_l;
float n_r;

void shockpotInit()
{

}

void shockpot100Hz()
{
    // pot_left, pot_right -- raw measuremens from ADC
    shockPots[0][start] = raw_adc_values.shock_left;
    shockPots[1][start] = raw_adc_values.shock_right;
    // float pot_speed_r = pot_speed(shockPots[0], RESOLUTION_FRONT, DELTA_FRONT, 10, start);
    // float pot_speed_l = pot_speed(shockPots[1], RESOLUTION_FRONT, DELTA_FRONT, 10, start);

    force(&n_l, &n_r, shockPots[0], shockPots[1], start);
    pot_speed(&pot_speed_l, &pot_speed_r);

    start = (start + N_REAR - 1) % N_REAR;
  //  SEND_FRONT_WHEEL_DATA(q_tx_can, 0, 0, n_rear_left, n_rear_right);
}
