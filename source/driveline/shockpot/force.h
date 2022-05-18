#ifndef __FORCE__
#define __FORCE__

// #define FTR_DRIVELINE_FRONT 1
// #define FTR_DRIVELINE_REAR 0
//Coefficients for second-oreder polynomial fit

#define A2          0.0206012
#define A1          0.19403
#define A0          0.281106

#define MAX_V        250         // maximum velocity for which we're able to return damping force value, mm/sec
#define VEL_SIZE     26          // size of the velocity storage

//Geometry of the car
typedef struct _Geometry {
    //Shock pot positions
    // Length
    float cd;           // Damper length, updatable

    float d_d;
    float d_w;
    float d_a;

    // Angles and their sines/cosines

    float fw_ob;
    float sin_ob_fw;
    float vert_ao;
    float oa_fa;
    float vert_fa;
    float ocd;
    float cos_ocd;
    float sin_ocd;
    float doc;

    // Displpacement for ARB
    float x_a;
} Geometry;

typedef struct _ForceParam {
    float f_damp;       // Damping force
    float f_total;      // Total damper force
    float n;            // Normal force of the wheel
    float v;            // Damper velocity

} ForceParam;

// typedef struct _ADCconv {
//     // Model for finding the length is CD = adc_0 + resolution*(ADC measurement)
//     const float adc_0;
//     const float resolution;
// } ADCconv;

typedef struct _Wheel {
    Geometry geom;
    ForceParam param;
    // ADCconv adc;
} Wheel;

void _get_pot_speed_pos(int* x, Wheel* w, float delta_T, int n, int start);
void _get_damp_force (float *f_damp, float v, const float force_reb [VEL_SIZE], const float force_comp [VEL_SIZE]);
void _upadte_geometry (Geometry *g);
void _get_total_force (Wheel *w);
void _get_normal_force (Wheel *w, Wheel *w_other);
void _calc_pipeline(int* x_l, int* x_r, Wheel *w_l, Wheel *w_r, int start);
void normal_force(float* n_l, float* n_r, int* x_l, int* x_r, int start);

#define N_SAMPLE    10                  // Number of position measurements we're using to
                                        // determine the rate of change of the shockpot position (velocisty)
#define DELTA_T     1                   // Sampling period of the mictocontroller

// Defines for the front board
#if (FTR_DRIVELINE_FRONT)

    // Element leghts as defined in the document.
    #define OA          123
    #define OB          123
    #define OC          123
    #define OD          123
        
    // Angles and their sines/cosines as defined in the document
    #define FW_VERT     123
    #define OD_VERT     123
    #define COB         123
    #define COA         123

    #define GAMMA       2004.75     // Torsion coefficient
    #define S           5           // ARB leverage, m
    #define K           146.75      // Spring coefficient
    #define X_0         123         // Rest position of the spring, force is determied as:
                                    // F_spring = k * (cd - x_0)


    #define RESOLUTION    1  // convert ADC value to real numbers, resolution of the ADC, 
    #define ADC_0         0   // actual displacement when ADC shows 0, in  

#elif (FTR_DRIVELINE_REAR)
    // Element leghts as defined in the document.
    #define OA          123
    #define OB          123
    #define OC          123
    #define OD          123
        
    // Angles and their sines/cosines as defined in the document
    #define FW_VERT     123
    #define OD_VERT     123
    #define COB         123
    #define COA         123

    #define GAMMA         2004.75     // torsion coefficient, m*N/rad
    #define S             5           // ARB leverage, m
    #define K             146.75      // spring constant, N/m
    #define X_0           123


    #define RESOLUTION    1  // convert ADC value to real numbers, resolution of the ADC, 
    #define ADC_0         0   // actual displacement when ADC shows 0, in  

#endif


#endif