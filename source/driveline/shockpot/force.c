#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include "force.h"

/*  
    
    Function foe determining the force based on a
    simple model in the PDF. Some of the values need to 
    be double checked before applying to the system.

*/

Wheel left, right;

#if (FTR_DRIVELINE_FRONT)

    const float FORCE_COMP[VEL_SIZE] = {    // copy data from one of the csv files, gives the measured compression damping force for a 
            0,                     // particular configuration of valves
        266.907,
        347.769,
        391.315,
        428.215,
        464.746,
        488.642,
        511.53,
        534.269,
        557.547,
        579.819,
        591.2,
        600.593,
        610.17,
        620.067,
        629.761,
        638.914,
        646.8,
        654.686,
        662.572,
        670.458,
        678.211,
        686.843,
        696.301,
        705.759,
        715.218,
    };

    const float FORCE_REB[VEL_SIZE] = {    // copy data from one of the csv files, gives the measured rebound damping force for a 
            0,                     // particular configuration of valves
        -232.487,
        -290.239,
        -324.172,
        -349.805,
        -374.158,
        -394.209,
        -413.159,
        -433.588,
        -453.067,
        -469.533,
        -485.999,
        -497.792,
        -509.725,
        -521.864,
        -534.617,
        -543.42,
        -550.862,
        -558.305,
        -566.015,
        -574.208,
        -582.04,
        -589.937,
        -598.443,
        -606.955,
        -615.468,
    };


#elif (FTR_DRIVELINE_REAR)

    const float FORCE_COMP[VEL_SIZE] = {    // copy data from one of the csv files, gives the measured compression damping force for a 
            0,                     // particular configuration of valves
        266.907,
        347.769,
        391.315,
        428.215,
        464.746,
        488.642,
        511.53,
        534.269,
        557.547,
        579.819,
        591.2,
        600.593,
        610.17,
        620.067,
        629.761,
        638.914,
        646.8,
        654.686,
        662.572,
        670.458,
        678.211,
        686.843,
        696.301,
        705.759,
        715.218,
    };

    const float FORCE_REB[VEL_SIZE] = {    // copy data from one of the csv files, gives the measured rebound damping force for a 
            0,                     // particular configuration of valves
        -232.487,
        -290.239,
        -324.172,
        -349.805,
        -374.158,
        -394.209,
        -413.159,
        -433.588,
        -453.067,
        -469.533,
        -485.999,
        -497.792,
        -509.725,
        -521.864,
        -534.617,
        -543.42,
        -550.862,
        -558.305,
        -566.015,
        -574.208,
        -582.04,
        -589.937,
        -598.443,
        -606.955,
        -615.468,
    };
#endif




// Weights for the polynomilal fitting algorithm
const float WEIGHTS[] = {
    1., 0.951229, 0.904837, 0.860708, 0.818731, 0.778801, 0.740818,
    0.704688, 0.67032, 0.637628
};


void _get_pot_speed_pos(int* x, Wheel* w, float delta_T, int n, int start) {     // second order polynomial fitting
    // x - array of position sensor data, which is cyclically updated
    // start - index of the latest measurement
    // delta_T - time interval between measurements
    // resolution - conversion of ADC step to real length

    // static float b;             
    // static float s0;
    // static float s1;
    // static float s2;

    // b = 0;
    // s0 = 0;
    // s1 = 0;
    // s2 = 0;

    float b = 0;             
    float s0 = 0;
    float s1 = 0;
    float s2 = 0;

    for (int i = 0; i < n; i++) {
        s0 += x[(start + i) % n] * WEIGHTS[i];
        s1 += i * x[(start + i) % n] * WEIGHTS[i];
        s2 += i * i * x[(start + i) % n] * WEIGHTS[i];
    }
    
    b = A0 * s0 - A1 * s1 + A2 * s2;   

    w->param.v = RESOLUTION * b / delta_T;
    w->geom.cd = RESOLUTION * x[start] +  ADC_0;
}


/*
    Calculates damping force based on the data from dyno tests. The data was digitized and put into corresponding
    *.csv files where the force is interpolted for values 0, 10, 20, ..., 250. This basically implements
    first oreder interpolation.
*/
void _get_damp_force (float *f_damp, float v, const float force_reb [VEL_SIZE], const float force_comp [VEL_SIZE]) {
    int val= (int)(floor(fabs(v/10)));
    if (v >= MAX_V) {
        *f_damp = force_reb[VEL_SIZE-1];
        printf("Damper velocity overshoot (positive, rebound)\n");
    } else if (v <= -MAX_V) {
        *f_damp = force_comp[VEL_SIZE-1];
        printf("Damper velocity overshoot (negative, compresion)\n");
    } else if (v > 0) {
        *f_damp = (force_reb[val] * (val * 10 + 10 - v) + force_reb[val + 1] * (v - val * 10)) / 10;
    } else {
        *f_damp = (force_comp[val] * (v + val * 10 + 10) + force_comp[val + 1] * (-val * 10 - v)) / 10;
    }
}


void _upadte_geometry (Geometry *g) {
    // update gometry of a suspension node, the comments correspond to the
    // equation numbers in the description document

    g->cos_ocd = (OC*OC + g->cd*g->cd - OD*OD)/(2 * OC * g->cd); // 1

    g->sin_ocd = sqrt(1 - g->cos_ocd*g->cos_ocd); // 2
    
    g->d_d = OC * g->sin_ocd; // 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

    g->doc = fabs(asin(sin(g->ocd) * g->cd / OD)); // 4

    g->fw_ob = COB + g->doc - OD_VERT + FW_VERT; //5

    g->sin_ob_fw = sin(g->fw_ob); // 6

    g->d_w = OB * g->sin_ob_fw; // 7

    g->vert_ao = COA + g->doc + OD_VERT; // 8

    g->oa_fa = g->vert_fa - g->vert_ao; // 9

    g->d_a = OA * fabs(sin(g->oa_fa)); // 10

    g->x_a = OA * sin (g->vert_ao - M_PI_2); // 11
}

void _get_total_force (Wheel *w) {
    float f_spring_l = -K * (w->geom.cd - X_0);
    w->param.f_total = f_spring_l + w->param.f_damp;
}

void _get_normal_force  (Wheel *w,  Wheel *w_other) {

    w->param.n =    (w->param.f_total * w->geom.d_d + 
                        GAMMA * (w_other->geom.x_a - w->geom.x_a) *
                        w->geom.d_a / S / S) 
                    /(w->geom.d_w * cos(FW_VERT));

}

void _calc_pipeline(int* x_l, int* x_r, Wheel *w_l, Wheel *w_r, int start) {

    _get_pot_speed_pos(x_l, w_l, DELTA_T, N_SAMPLE, start);
    _get_pot_speed_pos(x_r, w_r, DELTA_T, N_SAMPLE, start);

    _get_damp_force (&(w_l->param.f_damp), w_l->param.v, FORCE_REB, FORCE_COMP);
    _get_damp_force (&(w_r->param.f_damp), w_r->param.v, FORCE_REB, FORCE_COMP);

    _upadte_geometry(&(w_l->geom));
    _upadte_geometry(&(w_r->geom));

    _get_total_force (w_l);
    _get_total_force (w_r);

    _get_normal_force (w_l, w_r);
    _get_normal_force (w_r, w_l);
}

void normal_force(float* n_l, float* n_r, int* x_l, int* x_r, int start) {
    _calc_pipeline(x_l, x_r, &left, &right, start);
    *n_l = left.param.n;
    *n_r = right.param.n;
}


/*
    This is the main calculation function. It adjusts sampling window according to, 
    basically, change in speed. If the speed changes quickly, the estimator use only
    first few measurements to reduce the delay. If the speed chages slowly, 
    it adds more points t0 increase precision. The parameter ERROR that regulates this should be tuned 
    to achieve the best performance

    Input: sequence of the last N measurements of ADC.
    Return: estimated speed of the shockpot

    front wheel: coming up...
*/

// float pot_speed(int* x, float error, float resolution, float delta, int n, int start) {     // implements adaptive windowing as described in (1)

//     float b = 0;
//     float b_old = 0;                 

//     for (int i = 5; i < n; i++) {               // initialize the loop over the window size
//         float s1 = 0;                           // variables for summation
//         float s2 = 0;
//         for (int j = 0; j<=i; j++) {
//             s1 += x[(start + j) % n];
//             s2 += j * x[(start + j) % n];
//         }
//         b = (i * s1 - 2 * s2) / (i + 1) / (i + 2) / i * 6;                  // implementing the formula from the paper.
//         for (int j = 1; j<=i; j++) {
//             if (fabs(x[(start + j) % n] - (x[start] - j * b)) > error) {
//                 return resolution * b_old / delta;      // returing the spring force + damping force
//             }
//         }
//         b_old = b;
//     }
//     return resolution * b_old / delta; 
// }