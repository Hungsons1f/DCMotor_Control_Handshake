/**
  ******************************************************************************
  * @file           : PID_Controller.h
  * @brief          : PID Controller
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
//#define Pregain								(150.0/1150.0)
#define Postgain							(45.0/150.0)
//#define  kpmotor 							0.6
//#define  kimotor 							0//0.3
//#define  kdmotor 							0//0.075
#define  time 								0.02
//#define set									1000.0

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
volatile int32_t Calibrated_PID (volatile int32_t *count);
volatile float PID(float setpoint, float measure, float kp, float ki, float kd);
void ClearPIDController ();
