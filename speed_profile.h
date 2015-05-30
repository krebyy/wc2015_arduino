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
#define KP_X 2
#define KD_X 0
#define KP_W 10//15 -- 800mm/s
#define KD_W 80//3

#define TS 10	// Tempo de atualização em [ms]

#define CNT_PER_1000MM 3560
#define CNT_PER_360DEG 928	// = ((2*pi*W_DISTANCE)*CNT_PER_1000MM)/(2*1000)          928

#define SENSOR_SCALE 1


/* Macros --------------------------------------------------------------------*/
#define MM_TO_COUNTS(mm)	((((int32_t)mm) * CNT_PER_1000MM) / 1000)
#define SPEEDX_TO_COUNTS(speed)	((CNT_PER_1000MM * ((int32_t)speed * 2) * TS) / 1000000)
#define ACCX_TO_COUNTS(acc)		(SPEEDX_TO_COUNTS((int32_t)acc / 2) / TS)
#define COUNTS_TO_MM(cnt)	((((int32_t)cnt) * 1000) / CNT_PER_1000MM)

#define DEG_TO_COUNTS(deg)	((((int32_t)deg) * CNT_PER_360DEG) / 360)
#define SPEEDW_TO_COUNTS(speed)	((CNT_PER_360DEG * ((int32_t)speed * 2) * TS) / 360000)
#define ACCW_TO_COUNTS(acc)		(SPEEDW_TO_COUNTS((int32_t)acc / 2) / TS)
#define COUNTS_TO_DEG(cnt)	((((int32_t)cnt) * 360) / CNT_PER_360DEG)


/* Protótipos das Funções --------------------------------------------------- */
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int32_t needToDecelerate(int32_t, int32_t, int32_t);
void resetProfile(void);


/* Variáveis externas --------------------------------------------------------*/
extern int32_t distanceLeft, distance_mm;
extern int32_t targetSpeedX, targetSpeedW;
extern int32_t endSpeedX, endSpeedW;
extern int32_t accX, decX, accW, decW;

extern bool onlyUseEncoderFeedback;
extern bool onlyUseGyroFeedback;
extern bool onlyUseSensorFeedback;



#endif /* SPEED_PROFILE_H_ */
