/**
  ******************************************************************************
  * @file    wc2015/src/sensores.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    27-Abril-2015
  * @brief   Fun��es para controle de velocidade dos motores
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_profile.h"


/* Vari�veis privadas --------------------------------------------------------*/
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t encoderChange = 0, encoderCount = 0;
int32_t leftEncoderOld = 0, rightEncoderOld = 0;
int32_t leftEncoderCount = 0, rightEncoderCount = 0;
int32_t distance = 0;

int32_t rotationalFeedback = 0;
int32_t oldPosErrorX = 0, posErrorX = 0;
int32_t oldPosErrorW = 0, posErrorW = 0;

int32_t oldSensorError = 0;

int32_t curSpeedX = 0, curSpeedW = 0;

int32_t bufferLFT[201];
uint8_t index_buffer_lft = 1;
//uint8_t c_aux = 0;

int32_t bufferDistances[20] = {0};
int32_t bufferSpeedsWm[20] = {0};
uint8_t index_buffer_sector = 0;
int32_t accumulatorSpeedW = 0, numSpeedW = 0;

int32_t bufferSpeedXout[SIZE_BUFFER_SECTORS] = {0};
int32_t bufferSpeedWout[SIZE_BUFFER_SECTORS] = {0};


/* Vari�veis externas --------------------------------------------------------*/
int32_t distanceLeft = 0, distance_mm = 0;
int32_t targetSpeedX = 0, targetSpeedW = 0;
int32_t endSpeedX = 0, endSpeedW = 0;
int32_t accX = 0, decX = 0, accW = 0, decW = 0;

bool useEncoderFeedback = false;
bool useSensorFeedback = true;

uint8_t num_run = SEARCH_RUN;

int32_t buf_temp[2 * SIZE_BUFFER_SECTORS];




void speedProfile(void)
{
	getEncoderStatus();		// Leitura dos encoders e atualiza��o da dist�ncia
	updateCurrentSpeed();	// Atualiza os setpoints de velocidades do speedProfile
	calculateMotorPwm();	// Controlador de velocidade dos motores

	manageRuns();	// Tratamento das voltas (searchRun, fastRun1 e fastRun2)
}


void getEncoderStatus(void)
{
	int32_t leftEncoder, rightEncoder;

	leftEncoder = -encoderEsquerda.read();
	rightEncoder = -encoderDireita.read();

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	encoderChange = (leftEncoderChange + rightEncoderChange) / 2;

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;

	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount =  (leftEncoderCount + rightEncoderCount) / 2;

	distanceLeft -= encoderChange;// update distanceLeft
	distance += encoderChange;
	distance_mm = COUNTS_TO_MM(distance);
}


void updateCurrentSpeed(void)
{
	// Calcula para saber se chegou o momento de desacelerar
	if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
	{
		targetSpeedX = endSpeedX;
		targetSpeedW = endSpeedW;
	}

	// Gera o speedProfile do SpeedX (movimento translacional)
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += accX;
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= decX;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}

	// Gera o speedProfile do SpeedW (movimento rotacional)
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}
}


void calculateMotorPwm(void) // encoder PD controller
{
	int32_t sensorFeedback;

	int32_t encoderFeedbackX, encoderFeedbackW;
	int32_t posPwmX, posPwmW;

	rotationalFeedback = 0;

    // Feedbacks dos encoders
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	accumulatorSpeedW += encoderFeedbackW;
	numSpeedW++;

	// Leitura dos sensores de linha
	sensorFeedback = getSensorError();
	if (sensorFeedback == INFINITO) sensorFeedback = oldSensorError;
	oldSensorError = sensorFeedback;
	sensorFeedback /= SENSOR_SCALE;
	if (num_run != SEARCH_RUN) sensorFeedback /= PARAM_SCALE_SENSOR;

	// Habilita os feedbacks selecionados
	if (useEncoderFeedback == true) rotationalFeedback += encoderFeedbackW;
	if (useSensorFeedback == true) rotationalFeedback += sensorFeedback;

	// Calculo do erro
	posErrorX += curSpeedX - encoderFeedbackX;
	if (num_run == SEARCH_RUN) posErrorW = curSpeedW - rotationalFeedback;
	else if (flag_run < RUN_OK) posErrorW += curSpeedW - rotationalFeedback;
	else posErrorW = 0;

	// Controladores PDs para ambos motores
	posPwmX = KP_X * posErrorX + KD_X * (posErrorX - oldPosErrorX);
	if (num_run == SEARCH_RUN)
	{	// Seguidor de linha
		posPwmW = ((posErrorW * PARAM_PID_KP) / 128) +  (((posErrorW - oldPosErrorW) * PARAM_PID_KD) / 128);
	}
	else
	{	// SpeedProfile
		posPwmW = KP_W * posErrorW + KD_W * (posErrorW - oldPosErrorW);
	}

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	// Aciona os motores
	setMotores(posPwmX - posPwmW, posPwmX + posPwmW);
}


int32_t needToDecelerate(int32_t dist, int32_t curSpd, int32_t endSpd)
{
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0

	return (abs((curSpd*curSpd - endSpd*endSpd) / (2*dist)));
	//calculate deceleration rate needed with input distance, input current
	//speed and input targetspeed to determind if the deceleration is needed
	//use equaltion 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/(2*S)
}


void resetProfile(void)
{
	curSpeedX = curSpeedW = 0;
	posErrorX = posErrorW = 0;
	oldPosErrorX = oldPosErrorW = 0;
	distanceLeft = 0;
}


void manageRuns(void)
{
	static int32_t oldDistance = 0;

	switch (num_run)
	{
		case SEARCH_RUN:	// Corrida de de reconhecimento ********************
			// Registra a dist�ncia do trecho e o SpeedW_m�dio
			if (valid_marker == true)
			{
				bufferDistances[index_buffer_sector] = distance - oldDistance;
				bufferSpeedsWm[index_buffer_sector] = accumulatorSpeedW / numSpeedW;
				oldDistance = distance;

				index_buffer_sector++;
				accumulatorSpeedW = 0;
				numSpeedW = 0;
				valid_marker = false;
			}

			// Quando o rob� parar na linha de chegada: grava os dados do speedProfile
			if (flag_run == GOAL_OK && curSpeedX == 0)
			{
				// Cocatena os buffers e grava na flash
				uint32_t count = 0;
				memcpy(&buf_temp[count], bufferDistances, 4 * SIZE_BUFFER_SECTORS);
				count += SIZE_BUFFER_SECTORS;
				memcpy(&buf_temp[count], bufferSpeedsWm, 4 * SIZE_BUFFER_SECTORS);
				setMotores(0, 0);

				// Atualiza o estado
				flag_run = RUN_OK;
				valid_marker = false;
			}
			break;


		case FAST_RUN1:	// Corrida r�pida 1 ************************************
			if (valid_marker == true && flag_run != PAUSE && flag_run != GOAL_OK)
			{
				index_buffer_sector++;
				changeSpeedProfile();

				valid_marker = false;
			}

			if (flag_run == GOAL_OK && curSpeedX == 0)
			{
				setMotores(0, 0);
				flag_run = RUN_OK;
			}
			break;

		case FAST_RUN2:	// Corrida r�pida 2 ************************************
			if (valid_marker == true && flag_run != PAUSE && flag_run != GOAL_OK)
			{
				index_buffer_sector++;
				changeSpeedProfile();

				valid_marker = false;
			}

			if (flag_run == GOAL_OK && curSpeedX == 0)
			{
				setMotores(0, 0);
				flag_run = RUN_OK;
				num_run = STOP;
			}
			break;
	}
}


// Calcula os par�metros do speedProfile a partir dos dados dos trechos
void calculateSpeedProfile(int32_t topSpeedX, int32_t accC)
{
	// Calculo dos parametros do speedProfile
	for (uint8_t i = 0; i <= index_buffer_sector; i++)
	{
		if (i == SIZE_BUFFER_SECTORS) break;

		if (abs(bufferSpeedsWm[i]) < MINIMAL_SX_STRAIGHT)
		{	// Reta
			bufferSpeedXout[i] = SPEEDX_TO_COUNTS(topSpeedX);
			bufferSpeedWout[i] = 0;
		}
		else
		{	// Curva
			float ray = ((float)SPEEDX_TO_COUNTS(PARAM_SPEEDX_MED) / (float)bufferSpeedsWm[i]);
			bufferSpeedXout[i] = (int32_t)(sqrt(ACCC_TO_COUNTS(accC) * abs(ray * W_2)));
			if (bufferSpeedXout[i] > SPEEDX_TO_COUNTS(topSpeedX)) bufferSpeedXout[i] = SPEEDX_TO_COUNTS(topSpeedX);
			bufferSpeedWout[i] = (int32_t)(bufferSpeedXout[i] / ray);
		}
	}

	index_buffer_sector = 0;
}


// Altera os par�metros do speedProfile de acordo com os trechos
void changeSpeedProfile(void)
{
	targetSpeedX = bufferSpeedXout[index_buffer_sector];
	endSpeedX = bufferSpeedXout[index_buffer_sector + 1];

	targetSpeedW = bufferSpeedWout[index_buffer_sector];
	endSpeedW = bufferSpeedWout[index_buffer_sector + 1];

	distanceLeft = bufferDistances[index_buffer_sector];
	//if (targetSpeedW != 0) distanceLeft *= 2;  *** Adicionar ganho de distancia !!!!!!!
}


// L� os valores salvos na flash e carrega nos respectivos buffers
void updateBufferSpeedProfile(void)
{
	uint32_t buf[SIZE_BUFFER_SECTORS * 2];
	uint32_t count = 0;

	for (uint16_t i = 0; i < (SIZE_BUFFER_SECTORS * 2); i++)
	{
		EEPROM.get(i * 4, buf[i]);
	}

	memcpy(bufferDistances, &buf[count], 4 * SIZE_BUFFER_SECTORS);
	count += SIZE_BUFFER_SECTORS;
	memcpy(bufferSpeedsWm, &buf[count], 4 * SIZE_BUFFER_SECTORS);

	index_buffer_sector = SIZE_BUFFER_SECTORS;
}

