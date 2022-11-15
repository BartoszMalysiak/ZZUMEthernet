/**
 * @file LQIRRegulator.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "regulators.h"


void lqr_regulator_init(float *wzmocnienia, float *war_zadane) {

    for(int wzmoc_k = 0; wzmoc_k < INPUT_NUMBER; wzmoc_k++)
    {
        StdLqr[wzmoc_k].K = wzmocnienia[wzmoc_k];
        StdLqr[wzmoc_k].wartosc_zadana = war_zadane[wzmoc_k];
    }
};

int LQR_calculate(regulation_algorithm regulator, float control_value, float *feedback) {
    
    float control;
    float war_sum;
    float war_k[INPUT_NUMBER];

    for(int input_number = 0; input_number < INPUT_NUMBER; input_number++)
    {
        war_k[input_number] = (feedback[input_number] - StdLqr[input_number].wartosc_zadana) * StdLqr[input_number].K;
        war_sum += war_k[input_number];
    }

    control = control_value - war_sum;
    if(control > 1) control = 1;

    return control;
};



