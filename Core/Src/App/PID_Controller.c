/**
  ******************************************************************************
  * @file           : PID_Controller.c
  * @brief          : PID Controller
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "PID_Controller.h"

/* Private variables ---------------------------------------------------------*/
extern float setpoint;
extern float kp;
extern float ki;
extern float kd;

volatile float e0 = 0;
volatile float e1 = 0;
volatile float e2 = 0;
volatile float u0 = 0;
volatile float u1 = 0;

/* Private function -----------------------------------------------*/
volatile int32_t Calibrated_PID (volatile int32_t *count)
{
	float synchrocount = (float)*count;//*Pregain;
	float synchroduty;
	synchroduty = PID(setpoint, synchrocount, kp, ki, kd);
	int32_t duty = (int32_t)(synchroduty*Postgain);
	return duty;
}

volatile float PID(float setpoint, float measure, float kp, float ki, float kd)
{
	float temp = 0;
	e2 = e1;
	e1 = e0;
	e0 = setpoint - measure;
	u1 = u0;
	u0 = (u1 + kp*(e0 - e1) + ki*time*(e0 + e1)/2.0 + kd*(e0 - 2*e1 + e2)/time);
	//if (u0 < -150 ) u0 = -150;
	//if (u0>150 ) u0 =150;
	temp = u0;
	return temp;
}

void ClearPIDController ()
{
	e0 = 0;
	e1 = 0;
	e2 = 0;
	u0 = 0;
	u1 = 0;
}
