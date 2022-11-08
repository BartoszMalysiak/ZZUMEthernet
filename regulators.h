/*
 * regulators.h
 *
 *  Created on: Nov 8, 2022
 *      Author: Arkadiusz
 */

#ifndef INC_REGULATORS_H_
#define INC_REGULATORS_H_

typedef enum {
	PID_regulator = 0,				// Pełny regulator PID
	PI_regulator = 1,				// Regulator w wersji PI
	PD_regulator = 2,				// Regulator w wersji PD
	P_regulator = 3					// Regulator w wersji P
}regulation_algorithm;

// Strukltura zawierająca parametry algorytmów regulacji
typedef struct {
	float previous_error; 		//Poprzedni błąd dla członu różniczkującego
	float total_error;			//Suma uchybów dla członu całkującego
	float Tp;					//Czas próbkowania

	// regulatory: PID, PI, PD, P
	float Kp;					//Wzmocnienie członu proporcjonalnego
	float Ti;					//Czas całkowania
	float Td;					//Czas różniczkowania
	int anti_windup_limit;		//Anti-Windup - ograniczenie członu całkującego
}regulator_structure;

// Funkcje do obsługi regulatorów PID, PI, PD, P

void regulator_init(regulator_structure *pid_regulator, float Kp_init, float Ti_init, float Td_init, int anti_windup_limit_init, float histerezis_init, float deadzone_init, int three_pos_reg_controlvalue_init);

void regulator_reset(regulator_structure *pid_regulator);

int PID_calculate(regulator_structure *pid_regulator, regulation_algorithm regulator, float set_value, float process_variable);

#endif /* INC_REGULATORS_H_ */
