// Do not remove the include below
#include "wc2015_arduino.h"

// Configuraï¿½ï¿½o dos Encoders -------------------------------------
Encoder encoderEsquerda(B_ENC_E, A_ENC_E);
Encoder encoderDireita(A_ENC_D, B_ENC_D);



/* Vari�veis Globais ---------------------------------------------------------*/
bool run = false;


// Protï¿½tipos das funï¿½ï¿½es ----------------------------------------



// Inicializaï¿½ï¿½o dos pinos ---------------------------------------
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


	// Caso inicie o robô com SW1 pressionado: alteração dos parâmetros pelas rodinhas
	if (digitalRead(SW1) == HIGH)
	{
		while (digitalRead(SW1) == HIGH);
		delay(500);

		flag_run = 0;
		num_run = FAST_RUN1;

	}

	while (digitalRead(SW1)  == LOW)
	{
	}

	delay(1000);


	// Seleciona os parâmetros da respectiva corrida (searchRun, fastRun1 e fastRun2)
	initializeRun();

	encoderEsquerda.write(0);
	encoderDireita.write(0);
	run = true;
}


// LOOP principal do programa ------------------------------------
void loop()
{
	unsigned long t0;

	t0 = micros();

	mainSwitch();
	if (run == true) speedProfile();

	// Tempo de atualizaï¿½ï¿½o do controle = 10ms
	while((micros() - t0) < 10000);
}


void mainSwitch(void)
{
	switch (num_run)
	{
		case SEARCH_RUN:	// Corrida de de reconhecimento ****************
			if (flag_run == RUN_OK)
			{
				run = false;
				recordSectors();

				calculateSpeedProfile(PARAM_TOPSPEED1, PARAM_ACCC1);
				resetProfile();

				num_run = FAST_RUN1;
				flag_run = PAUSE;
			}
			break;


		case FAST_RUN1:	// Corrida rápida 1 ********************************
			if (run == false)
			{
				while (digitalRead(SW1) == LOW);

				delay(1000);

				useEncoderFeedback = true;
				useSensorFeedback = true;

				valid_marker = false;

				changeSpeedProfile();

				flag_run = 0;
				run = true;
			}

			if (flag_run == RUN_OK)
			{
				run = false;
				calculateSpeedProfile(PARAM_TOPSPEED2, PARAM_ACCC2);
				resetProfile();

				num_run = FAST_RUN2;
				flag_run = PAUSE;
			}
			break;


		case FAST_RUN2:	// Corrida rápida 2 ********************************
			if (run == false)
			{
				while (digitalRead(SW1) == LOW);

				delay(1000);

				useEncoderFeedback = true;
				useSensorFeedback = true;

				valid_marker = false;

				changeSpeedProfile();

				flag_run = 0;
				run = true;
			}
			break;


		case STOP:
			run = false;
			break;
	}
}



void initializeRun(void)
{
	accX = decX = 1;//ACCX_TO_COUNTS(500);//param_accX);
	accW = decW = 1;

	switch (num_run)
	{
		case SEARCH_RUN:	// Corrida de de reconhecimento ********************
			useEncoderFeedback = false;
			useSensorFeedback = true;

			targetSpeedX = SPEEDX_TO_COUNTS(PARAM_SPEEDX_MED);
			distanceLeft = MM_TO_COUNTS(10000);
			break;


		case FAST_RUN1:	// Corrida rápida 1 ************************************
			useEncoderFeedback = true;
			useSensorFeedback = true;

			updateBufferSpeedProfile();
			calculateSpeedProfile(PARAM_TOPSPEED1, PARAM_ACCC1);
			changeSpeedProfile();
			break;


		case FAST_RUN2:	// Corrida rápida 2 ************************************
			useEncoderFeedback = true;
			useSensorFeedback = true;

			updateBufferSpeedProfile();
			calculateSpeedProfile(PARAM_TOPSPEED2, PARAM_ACCC2);
			changeSpeedProfile();
			break;
	}
}


void recordSectors(void)
{
	for (uint16_t i = 0; i < (SIZE_BUFFER_SECTORS * 2); i++)
	{
		EEPROM.put(i * 4, buf_temp[i]);
	}

}
