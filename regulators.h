/*
 * regulators.h
 *
 *  Created on: Nov 8, 2022
 *      Author: Arkadiusz
 */

#ifndef INC_REGULATORS_H_
#define INC_REGULATORS_H_

#define INPUT_NUMBER 3
typedef enum {
	PID_regulator   = 0,				/* Regualtor w wersji P */
	PI_regulator    = 1,				/* Regulator w wersji PI */
	PD_regulator    = 2,				/* Regulator w wersji PD */
	P_regulator     = 3,				/* Regulator w wersji P */
	LQI_regulator   = 4,				/* Regulator w wersji LQI */
	LQR_regulator   = 5					/* Regulator w wersji LQR */
}regulation_algorithm;

/**
 * @brief Struktura zawierająca parametry algorytmów regulacji
 * 
 */
typedef struct {
	float previous_error; 		/* Poprzedni błąd dla członu różniczkującego */
	float total_error;			/* Suma uchybów dla członu całkującego */
	float Tp;					/* Czas próbkowania */

	/* regulatory: PID, PI, PD, P */
	float Kp;					/* Wzmocnienie członu proporcjonalnego */
	float Ti;					/* Czas całkowania */
	float Td;					/* Czas różniczkowania */
	int anti_windup_limit;		/* Anti-Windup - ograniczenie członu całkującego */
}PIDRegulator;

/**
 * @brief Struktura zawierająca parametry regulatora LQR/LQI
 * 
 */
typedef struct {
	float K;					/* Wzmocnienie stanu */
	float feedback;				/* Wartosc stanu otrzymana ze sprzezenia zwrotnego */
	float wartosc_zadana;		/* Wartosc zadana stanu */
}LQRRegulator;



/**
 * @brief Inicjalizacja regulatora
 * 
 * @param pid_regulator 
 * @param Kp_init 
 * @param Ti_init 
 * @param Td_init 
 * @param anti_windup_limit_init 
 * @param histerezis_init 
 * @param deadzone_init 
 * @param three_pos_reg_controlvalue_init 
 */
void regulator_init(PIDRegulator *pid_regulator, float Kp_init, float Ti_init, float Td_init, int anti_windup_limit_init, float histerezis_init, float deadzone_init, int three_pos_reg_controlvalue_init);

/**
 * @brief Funkcja resetująca regulator
 * 
 * @param pid_regulator 
 */
void regulator_reset(PIDRegulator *pid_regulator);

/**
 * @brief Obliczanie wartości sterowania
 * 
 * @param pid_regulator 
 * @param regulator 
 * @param set_value 
 * @param process_variable 
 * @return int 
 */
int PID_calculate(PIDRegulator *pid_regulator, regulation_algorithm regulator, float set_value, float process_variable);

/**
 * @brief 
 * 
 * @param wzmocnienia 
 */
void lqr_regulator_init(float *wzmocnienia);

/**
 * @brief 
 * 
 * @param regulator 
 * @param control_value 
 * @param feedback 
 * @return int 
 */
int LQR_calculate(regulation_algorithm regulator, float control_value, float *feedback);

extern LQRRegulator StdLqr[INPUT_NUMBER];

#endif /* INC_REGULATORS_H_ */
