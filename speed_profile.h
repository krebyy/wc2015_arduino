/*
 * speed_profile.h
 *
 *  Created on: 20/05/2015
 *      Author: Kleber
 */

#ifndef SPEED_PROFILE_H_
#define SPEED_PROFILE_H_


/* Includes ------------------------------------------------------------------*/
#include "wc2015_arduino.h"
#include "motores.h"

/* Constantes ----------------------------------------------------------------*/
#define KP_X 1
#define KD_X 2
#define KP_W 1
#define KD_W 2

#define TS 10	// Tempo de atualiza��o em [ms]

#define CNT_PER_1000MM 3560
#define CNT_PER_360DEG 928	// = ((2*pi*W_DISTANCE)*CNT_PER_1000MM)/(2*1000)          928
#define W_DISTANCE	83

#define SENSOR_SCALE 1

#define MINIMAL_SX_STRAIGHT 5

#define SIZE_BUFFER_SECTORS		20

#define SEARCH_RUN		0
#define FAST_RUN1		1
#define FAST_RUN2		2
#define PAUSE			9
#define STOP			10

#define GOAL_OK			4
#define RUN_OK			5
#define WAIT			6

/* Macros --------------------------------------------------------------------*/
#define MM_TO_COUNTS(mm)	((((int32_t)mm) * CNT_PER_1000MM) / 1000)
#define SPEEDX_TO_COUNTS(speed)	((CNT_PER_1000MM * ((int32_t)speed * 2) * TS) / 1000000)
#define ACCX_TO_COUNTS(acc)		(SPEEDX_TO_COUNTS((int32_t)acc / 2) / TS)
#define COUNTS_TO_MM(cnt)	((((int32_t)cnt) * 1000) / CNT_PER_1000MM)

#define DEG_TO_COUNTS(deg)	((((int32_t)deg) * CNT_PER_360DEG) / 360)
#define SPEEDW_TO_COUNTS(speed)	((CNT_PER_360DEG * ((int32_t)speed * 2) * TS) / 360000)
#define ACCW_TO_COUNTS(acc)		(SPEEDW_TO_COUNTS((int32_t)acc / 2) / TS)
#define COUNTS_TO_DEG(cnt)	((((int32_t)cnt) * 360) / CNT_PER_360DEG)

#define ACCC_TO_COUNTS(acc) (float)((float)acc / 702.2472f)

#define W_2		(float)((float)MM_TO_COUNTS(W_DISTANCE) / 2.0f)


/* Prot�tipos das Fun��es --------------------------------------------------- */
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int32_t needToDecelerate(int32_t, int32_t, int32_t);
void resetProfile(void);

void manageRuns(void);
void calculateSpeedProfile(int32_t topSpeedX, int32_t accC);
void changeSpeedProfile(void);
void updateBufferSpeedProfile(void);


/* Vari�veis externas --------------------------------------------------------*/
extern int32_t distanceLeft, distance_mm;
extern int32_t targetSpeedX, targetSpeedW;
extern int32_t endSpeedX, endSpeedW;
extern int32_t accX, decX, accW, decW;
extern bool useEncoderFeedback;
extern bool useGyroFeedback;
extern bool useSensorFeedback;
extern uint8_t num_run;
extern int32_t buf_temp[2 * SIZE_BUFFER_SECTORS];



#endif /* SPEED_PROFILE_H_ */
