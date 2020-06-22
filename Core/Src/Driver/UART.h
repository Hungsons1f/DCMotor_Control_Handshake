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

/* Private enums -------------------------------------------------------------*/
enum DataHeader  {Realtime = 0x01, Setpoint = 0x02, Measure = 0x03,
					Data = 0x04
					};
enum ControlHeader	{Run = 0x11, Stop = 0x12, Velocity = 0x13, Position = 0x14, ctrlSetpoint = 0x15, ctrlRealtime = 0x16,
						Kp = 0x17, Ki = 0x18, Kd = 0x19,
						Calib = 0x31,
						Nak = 0xF0, Ack = 0xF1};
enum FrameHeader  {STX = 0xFE, ETX = 0xFF, DLE = 0xFD};

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
uint8_t* Float2Char (float floatNum);
void TransmitCommand (uint8_t command, uint8_t haveData, float data);
void EchoReceived (uint8_t *rxbuff);
float Char2Float (uint8_t *charNum);
float ReceiveDataofHeader (uint8_t *rxbuffer);
uint8_t ReceiveAndHandshakeOrigin (uint8_t *buffer, uint8_t *instruction);
uint8_t ReceiveAndHandshake (uint8_t *buffer, uint8_t *instruction);
void TransmitDataOrigin (float realtime, float measure);
uint8_t TransmitAndHandshake (float *buffer, float realtime, float measure, float pwm);
void TransmitData (float realtime, float measure, float pwm);
uint8_t ReceiveAck (uint8_t *buffer);
