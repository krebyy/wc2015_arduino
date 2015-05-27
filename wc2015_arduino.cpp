// Do not remove the include below
#include "wc2015_arduino.h"

// Configuração dos Encoders -------------------------------------
Encoder encoderEsquerda(B_ENC_E, A_ENC_E);
Encoder encoderDireita(A_ENC_D, B_ENC_D);



// Variáveis do programa -----------------------------------------



// Protótipos das funções ----------------------------------------



// Inicialização dos pinos ---------------------------------------
void setup()
{
	pinMode(LINHA6, INPUT);
	pinMode(LINHA5, INPUT);
	pinMode(LINHA4, INPUT);
	pinMode(LINHA3, INPUT);
	pinMode(LINHA2, INPUT);
	pinMode(LINHA1, INPUT);
	pinMode(EMISSORES, OUTPUT);
	//pinMode(LED1, OUTPUT);
	//pinMode(LED2, OUTPUT);
	pinMode(0, INPUT);
	pinMode(1, OUTPUT);
	pinMode(PWM_E, OUTPUT);
	pinMode(IN1_E, OUTPUT);
	pinMode(IN2_E, OUTPUT);
	pinMode(PWM_D, OUTPUT);
	pinMode(IN1_D, OUTPUT);
	pinMode(IN2_D, OUTPUT);
	pinMode(SW1, INPUT);

	Serial.begin(115200);

	while (digitalRead(SW1) == LOW)
	{
		// funções para leitura/gravação dos parâmetros do robô
	}

	delay(1000);

	targetSpeedX = SPEEDX_TO_COUNTS(800);//param_speedX_med);
	accX = decX = ACCX_TO_COUNTS(1000);//param_accX);
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

	// Tempo de atualização do controle = 10ms
	while((micros() - t0) < 10000);
}
