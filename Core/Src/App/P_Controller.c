/**
  ******************************************************************************
  * @file           : P_Controller.c
  * @brief          : P Controller
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "P_Controller.h"

/* Private variables ---------------------------------------------------------*/
extern float setpoint;
extern float kp;


/* Private function -----------------------------------------------*/
volatile int32_t Calibrated_P (volatile int32_t *count)
{
	float synchrocount = (float)*count*PregainP;
	float synchroduty;
	synchroduty = Proportional(setpoint, synchrocount, kp);
	int32_t duty = (int32_t)(synchroduty*PostgainP);
	return duty;
}

volatile float Proportional(float setpoint, float measure, float kp)
{
	static volatile float e0 = 0;
	static volatile float u0 = 0;
	e0 = setpoint - measure;
	u0 = kp*e0;
	return u0;
}
