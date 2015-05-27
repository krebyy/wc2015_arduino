/*
 * motores.h
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */

#ifndef MOTORES_H_
#define MOTORES_H_


#include "Arduino.h"


const int PWM_E = 6;       // PWM do motor esquerdo
const int IN2_E = 7;       // IN2 da ponte H do motor esquerdo
const int IN1_E = 8;       // IN1 da ponte H do motor esquerdo
const int IN1_D = 9;       // IN1 da ponte H do motor direito
const int IN2_D = 10;      // IN2 da ponte H do motor direito
const int PWM_D = 11;      // PWM do motor direito


void setMotores(int pwm_esquerda, int pwm_direita);


#endif /* MOTORES_H_ */
