/*
 * sensores.cpp
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */

#include "sensores.h"


/**
  * @brief Realiza várias leituras dos sensores de linha e retorna a média
  * @param Nenhum
  * @return erro Valores negativos (delocado para direita), valores positivos
  * (deslocado para esquerda), INFINITO caso não tenha detectado linha
  */
int32_t getSensorError(void)
{
	int32_t erro = 0, soma = 0, n = 0;

	for(int i = 25; i <= 100; i += 5)
	{
		unsigned long t0 = micros();

		// Habilita os emissores por 100 us
		digitalWrite(EMISSORES, HIGH);
		while ((micros() - t0) < i);

		// Realiza a leitura de todos os sensores de linha. Os sensores
		// das extremidades possuem peso maior. No final é realizada
		// a média ponderada
		if (digitalRead(LINHA1) == LINHA)
		{
			soma += -1333;
			n++;
		}
		if (digitalRead(LINHA2) == LINHA)
		{
			soma += -667;
			n++;
		}
		if (digitalRead(LINHA3) == LINHA)
		{
			soma += -133;
			n++;
		}
		if (digitalRead(LINHA4) == LINHA)
		{
			soma += 133;
			n++;
		}
		if (digitalRead(LINHA5) == LINHA)
		{
			soma += 667;
			n++;
		}
		if (digitalRead(LINHA6) == LINHA)
		{
			soma += 1333;
			n++;
		}

		// Desabilita os emissores
		digitalWrite(EMISSORES, LOW);
		while ((micros() - t0) < (2 * i));
	}

	// Retorna a média ou retorna a constante INFINITO indicando
	// que nenhum sensor leu linha
	if (n != 0)
	{
		erro = soma / n;
	}
	else
	{
		erro = INFINITO;
	}

	return erro;
}
