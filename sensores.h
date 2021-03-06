/*
 * sensores.h
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */

#ifndef SENSORES_H_
#define SENSORES_H_


#include "wc2015_arduino.h"


#define LINHA 		LOW  	   // LOW: linha branca | HIGH: linha preta
#define INFINITO 	8888
#define MARKER_TH	2

const uint8_t LINHA6 = A0;     // Sensor de linha 6 (esquerda)
const uint8_t LINHA5 = A1;     // Sensor de linha 5
const uint8_t LINHA4 = A2;     // Sensor de linha 4
const uint8_t LINHA3 = A3;     // Sensor de linha 3
const uint8_t LINHA2 = A4;     // Sensor de linha 2
const uint8_t LINHA1 = A5;     // Sensor de linha 1 (direita)
const uint8_t L_MARK_R = 0;	   // Sensor de marca (esquerda)
const uint8_t R_MARK_R = 1;    // Sensor de marca (direita)
const uint8_t EMISSORES = 13;  // LEDs dos sensores de linha


int32_t getSensorError(void);
void readMarks(void);

/* Variáveis externas --------------------------------------------------------*/
extern bool valid_marker;
extern int32_t flag_run;

#endif /* SENSORES_H_ */
