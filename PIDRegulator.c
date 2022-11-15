/**
 * @file PIDRegulator.c
 * @author Arkadiusz, Bartosz (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "regulators.h"

void regulator_init(PIDRegulator *regulator, float Kp_init, float Ti_init, float Td_init, int anti_windup_limit_init, float hysteresis_init, float deadzone_init, int three_pos_reg_controlvalue_init) {
	regulator->previous_error = 0;
	regulator->total_error = 0;
	regulator->Tp = 0.75;

	regulator->Kp = Kp_init;
	regulator->Ti = Ti_init;
	regulator->Td = Td_init;

	regulator->anti_windup_limit = anti_windup_limit_init;
}

void regulator_reset(PIDRegulator *regulator) {
	regulator->total_error = 0;
	regulator->previous_error = 0;
}

int PID_calculate(PIDRegulator *pid_regulator, regulation_algorithm regulator, float set_value, float process_variable) {
	float error;
	float P, I, D;
	float max_total_error;

	error = set_value - process_variable;			//obliczenie uchybu

	pid_regulator->total_error += error;		//sumowanie uchybu

	max_total_error = (pid_regulator->anti_windup_limit*pid_regulator->Ti)/(pid_regulator->Kp*pid_regulator->Tp);

	//Anti-Windup - ograniczenie sumowania błędu
	if (pid_regulator->total_error > max_total_error) {
		pid_regulator->total_error = max_total_error;
	} else if (pid_regulator->total_error < -max_total_error) {
		pid_regulator->total_error = -max_total_error;
	}

	P = (float)(pid_regulator->Kp * error);																				//odpowiedź członu proporcjonalnego
	if (pid_regulator->Ti != 0){
		I = (float)(((pid_regulator->Kp/pid_regulator->Ti)*pid_regulator->total_error)*pid_regulator->Tp);				//odpowiedź członu całkującego
	} else {
		I = 0;
	}
	D = (float)((pid_regulator->Kp * pid_regulator->Td)*((error - pid_regulator->previous_error)/pid_regulator->Tp));	//odpowiedź członu różniczkującego

	//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	if (I > pid_regulator->anti_windup_limit) {
		I = pid_regulator->anti_windup_limit;
	}
	else if (I < -pid_regulator->anti_windup_limit) {
		I = -pid_regulator->anti_windup_limit;
	}

	//aktualizacja zmiennej z poprzednią wartością błędu
	pid_regulator->previous_error = error;

	switch (regulator) {
		case PID_regulator:
			return (int)(P + I + D);	//odpowiedź regulatora PID
			break;
		case PI_regulator:
			return (int)(P + I);		//odpowiedź regulatora PI
			break;
		case PD_regulator:
			return (int)(P + D);		//odpowiedź regulatora PD
			break;
		case P_regulator:
			return (int)(P);			//odpowiedź regulatora P
			break;
		default:
			return (int)0;				//odpowiedź domyślna
			break;
	}
}
