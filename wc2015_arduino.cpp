// Do not remove the include below
#include "wc2015_arduino.h"

// Configura��o dos Encoders -------------------------------------
Encoder encoderEsquerda(B_ENC_E, A_ENC_E);
Encoder encoderDireita(A_ENC_D, B_ENC_D);



// Vari�veis do programa -----------------------------------------



// Prot�tipos das fun��es ----------------------------------------



// Inicializa��o dos pinos ---------------------------------------
void setup()
{
	pinMode(LINHA6, INPUT);
	pinMode(LINHA5, INPUT);
	pinMode(LINHA4, INPUT);
	pinMode(LINHA3, INPUT);
	pinMode(LINHA2, INPUT);
	pinMode(LINHA1, INPUT);
	pinMode(EMISSORES, OUTPUT);
	pinMode(L_MARK_R, INPUT);
	pinMode(R_MARK_R, INPUT);
	pinMode(PWM_E, OUTPUT);
	pinMode(IN1_E, OUTPUT);
	pinMode(IN2_E, OUTPUT);
	pinMode(PWM_D, OUTPUT);
	pinMode(IN1_D, OUTPUT);
	pinMode(IN2_D, OUTPUT);
	pinMode(SW1, INPUT);


	while (digitalRead(SW1) == LOW)
	{
		// fun��es para leitura/grava��o dos par�metros do rob�
	}

	delay(1000);

	targetSpeedX = SPEEDX_TO_COUNTS(PARAM_SPEEDX_MED);		// PARAM_SPEEDX_MED;
	accX = decX = ACCX_TO_COUNTS(PARAM_ACCX);   // PARAM_ACCX);
	distanceLeft = MM_TO_COUNTS(13155);

	encoderEsquerda.write(0);
	encoderDireita.write(0);
}


// LOOP principal do programa ------------------------------------
void loop()
{
	unsigned long t0;

	t0 = micros();

	speedProfile();

	// Tempo de atualiza��o do controle = 10ms
	while((micros() - t0) < 10000);
}
