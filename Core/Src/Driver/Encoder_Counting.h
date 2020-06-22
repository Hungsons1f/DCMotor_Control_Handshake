/**
  ******************************************************************************
  * @file           : Encoder_Counting.h
  * @brief          : Encoder Capture driver
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
volatile int32_t EncoderCount (TIM_HandleTypeDef *timer);
volatile int32_t EncoderPosCount (TIM_HandleTypeDef *timer);
void ClearEncoderCount (TIM_HandleTypeDef *timer);
