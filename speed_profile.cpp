/*
 * speed_profile.cpp
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */


/* Includes ------------------------------------------------------------------*/
#include "speed_profile.h"


/* Variáveis privadas --------------------------------------------------------*/
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t encoderChange = 0, encoderCount = 0;
int32_t leftEncoderOld = 0, rightEncoderOld = 0;
int32_t leftEncoderCount = 0, rightEncoderCount = 0;
int32_t distance = 0;

int32_t oldPosErrorX = 0, posErrorX = 0;
int32_t oldPosErrorW = 0, posErrorW = 0;

int32_t oldSensorError = 0;

int32_t curSpeedX = 0, curSpeedW = 0;


/* Variáveis externas --------------------------------------------------------*/
int32_t distanceLeft = 0, distance_mm = 0;
int32_t targetSpeedX = 0, targetSpeedW = 0;
int32_t endSpeedX = 0, endSpeedW = 0;
int32_t accX = 0, decX = 0, accW = 0, decW = 0;

bool onlyUseEncoderFeedback = false;
bool onlyUseGyroFeedback = false;
bool onlyUseSensorFeedback = false;





void speedProfile(void)
{
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();
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
	encoderCount =  (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;

	//distanceLeft -= encoderChange;// update distanceLeft
	distance += encoderChange;
	distance_mm = COUNTS_TO_MM(distance);
}


void updateCurrentSpeed(void)
{
	if (targetSpeedW == 0)
	{
		if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
		{
			targetSpeedX = endSpeedX;
		}
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
	}
	else
	{
		if (needToDecelerate(distanceLeft, curSpeedW, endSpeedW) > decW)
		{
			targetSpeedW = endSpeedW;
		}
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
}


void calculateMotorPwm(void) // encoder PD controller
{
	int32_t rotationalFeedback;
	int32_t sensorFeedback;
	int32_t encoderFeedbackX, encoderFeedbackW;
	int32_t posPwmX, posPwmW;

    /* simple PD loop to generate base speed for both motors */
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	sensorFeedback = getSensorError();
	if (sensorFeedback == INFINITO) sensorFeedback = oldSensorError;
	oldSensorError = sensorFeedback;
	sensorFeedback /= SENSOR_SCALE;

	/*if(onlyUseGyroFeedback == true)
		rotationalFeedback = gyroFeedback;
	else if(onlyUseEncoderFeedback == true)
		rotationalFeedback = encoderFeedbackW;
	else
		rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;*/
	rotationalFeedback = sensorFeedback;

	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW = curSpeedW - rotationalFeedback;

	posPwmX = KP_X * posErrorX + KD_X * (posErrorX - oldPosErrorX);
	posPwmW = ((posErrorW * KP_W) / 128) + (((posErrorW - oldPosErrorW) * KD_W) / 128);

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

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
	curSpeedX = 0;
	curSpeedW = 0;
	encoderEsquerda.write(0);
	encoderDireita.write(0);

	leftEncoderCount = 0;
	rightEncoderCount = 0;
	encoderCount = 0;

	posErrorX = 0;
	posErrorW = 0;
	oldPosErrorX = 0;
	oldPosErrorW = 0;

	distance = 0;
	distance_mm = 0;
}

