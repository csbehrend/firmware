#include "tv_data.h"

/* Variable Definitions */
double RE;
double s[2];
double l[2];
double mu_factor;
double k_limit;

void tv_data_init()
{
    k_limit = 10.0;
    mu_factor = 1.0;
    l[0] = 1.0;
    s[0] = 1.0;
    l[1] = 2.0;
    s[1] = 2.0;
    RE = 0.03;
}