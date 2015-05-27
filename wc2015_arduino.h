// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _wc2015_arduino_H_
#define _wc2015_arduino_H_
#include "Arduino.h"
//add your includes for the project wc2015_arduino here
#include "Encoder.h"
#include "motores.h"
#include "sensores.h"
#include "speed_profile.h"
#include <stdint.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project wc2015_arduino here

/* Constantes ----------------------------------------------------------------*/
// Definição dos pinos no modo MICROMOUSE
const uint8_t LED1 = 0;		// LED1
const uint8_t LED2 = 1;		// LED2
const uint8_t B_ENC_E = 2;	// Encoder do motor esquerdo (sinal B)
const uint8_t A_ENC_D = 3;	// Encoder do motor direito (sinal A)
const uint8_t A_ENC_E = 4;	// Encoder do motor esquerdo (sinal A)
const uint8_t B_ENC_D = 5;	// Encoder do motor direito (sinal B)
const uint8_t SW1 = 12;		// Botão SW1


/* Variáveis Externas --------------------------------------------------------*/
extern Encoder encoderEsquerda;
extern Encoder encoderDireita;


//Do not add code below this line
#endif /* _wc2015_arduino_H_ */
