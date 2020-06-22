/**
  ******************************************************************************
  * @file           : P_Controller.h
  * @brief          : P Controller
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define PregainP							(150.0/1150.0)
#define PostgainP							(45.0/150.0)

//#define set									100.0
//#define P									2

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
volatile int32_t Calibrated_P (volatile int32_t *count);
volatile float Proportional(float setpoint, float measure, float kp);
