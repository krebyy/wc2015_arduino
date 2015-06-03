/*
 * sensores.cpp
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */

#include "sensores.h"

/* Vari�veis externas --------------------------------------------------------*/
bool valid_marker = false;
int32_t flag_run = 0;


/**
  * @brief Realiza v�rias leituras dos sensores de linha e retorna a m�dia
  * @param Nenhum
  * @return erro Valores negativos (delocado para direita), valores positivos
  * (deslocado para esquerda), INFINITO caso n�o tenha detectado linha
  */
int32_t getSensorError(void)
{
	int32_t erro = 0, soma = 0, n = 0;

	for(uint8_t i = 25; i <= 100; i += 5)
	{
		unsigned long t0 = micros();

		// Habilita os emissores por 100 us
		digitalWrite(EMISSORES, HIGH);
		while ((micros() - t0) < i);

		// Realiza a leitura de todos os sensores de linha. Os sensores
		// das extremidades possuem peso maior. No final � realizada
		// a m�dia ponderada
		if (digitalRead(LINHA1) == LINHA)
		{
			soma += -333;
			n++;
		}
		if (digitalRead(LINHA2) == LINHA)
		{
			soma += -167;
			n++;
		}
		if (digitalRead(LINHA3) == LINHA)
		{
			soma += -33;
			n++;
		}
		if (digitalRead(LINHA4) == LINHA)
		{
			soma += 33;
			n++;
		}
		if (digitalRead(LINHA5) == LINHA)
		{
			soma += 167;
			n++;
		}
		if (digitalRead(LINHA6) == LINHA)
		{
			soma += 333;
			n++;
		}

		if (i == 100)
		{
			readMarks();
		}

		// Desabilita os emissores
		digitalWrite(EMISSORES, LOW);
		while ((micros() - t0) < (2 * i));
	}

	// Retorna a m�dia ou retorna a constante INFINITO indicando
	// que nenhum sensor leu linha
	if (n != 0)
	{
		erro = soma / n;
	}
	else
	{
		erro = INFINITO;
	}

	for (uint8_t i = 0; i < 2; i++)
	{
		unsigned long t0 = micros();

		// Habilita os emissores por 100 us
		digitalWrite(EMISSORES, HIGH);
		while ((micros() - t0) < 100);

		readMarks();

		// Desabilita os emissores
		digitalWrite(EMISSORES, LOW);
		while ((micros() - t0) < 1000);	// Tempo de atualiza��o da leitura das marcas = 1ms
	}

	return erro;
}


/**
  * @brief
  * @param
  * @return
  */
void readMarks(void)
{
	static int32_t marker_corner = 0, marker_start_goal = 0;
	static bool marker_intersection = false, fmarker = false;

	// DetecÃ§Ã£o das marcas de partida/chegada
	if (digitalRead(R_MARK_R) == HIGH)
	{
		marker_start_goal++;
	}
	else if (marker_start_goal > 0)
	{
		marker_start_goal--;
	}

	// corner and start/goal marker detection
	if (digitalRead(L_MARK_R) == HIGH)
	{
		marker_corner++;
	}
	else if (marker_corner > 0)
	{
		marker_corner--;
	}

	// detection of intersection, both marker - ignore intersection
	if (marker_start_goal > 1 && marker_corner > 1)
	{
		marker_intersection = true;
	}
	if (marker_intersection == true && marker_start_goal == 0 && marker_corner == 0)
	{
		marker_intersection = false;
	}


	// corner marker check
	if (marker_intersection == true)
	{
		fmarker = false;
	}
	if (marker_intersection == false && fmarker == false && marker_corner > MARKER_TH)
	{	// corner marker detect
		fmarker = true;
	}
	if (marker_intersection == false && fmarker == true && marker_corner == 0)
	{	// corner marker fix
		fmarker = false;

		valid_marker = true;
	}

	// start/goal marker check
	if (flag_run == 0 && marker_start_goal > MARKER_TH)
	{	// start marker detect
		flag_run = 1;
	}
	if (flag_run == 1 && marker_start_goal == 0)
	{	// start marker fix
		flag_run=2;

		valid_marker = true;
	}
	if (flag_run == 2 && marker_start_goal > MARKER_TH)
	{	// goal marker detect
		flag_run = 3;
	}
	if (flag_run == 3 && marker_intersection == true)
	{	// ignore intersection
		flag_run = 2;
	}
	if (flag_run == 3 && marker_start_goal == 0)
	{	// goal marker fix
		flag_run = GOAL_OK;
		distanceLeft = MM_TO_COUNTS(150);
		endSpeedX = 0;

		valid_marker = true;
	}
}
