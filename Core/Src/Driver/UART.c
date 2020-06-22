/**
  ******************************************************************************
  * @file           : Encoder_Counting.c
  * @brief          : Encoder Capture driver
  ******************************************************************************
  ******************************************************************************
  */

/* Includes-------------------------------------------------------------------*/
#include "UART.h"

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim6;

const uint8_t txbuff[][30] = {
		"\nMotor running...",
		"\nMotor Stopped!",
		"\nVelocity Mode",
		"\nPosition Mode",
		"\nSetpoint Set!"
};

uint8_t haveFrame = 0;
static uint8_t rxindex = 0, rxbytefield[6], rxbuffer[11];
uint8_t txack = 1, txnak = 0, txindex = 0;

uint8_t rxnak = 0;

/* Private function -----------------------------------------------*/
void TransmitCommand (uint8_t command, uint8_t haveData, float data)
{
	uint8_t stx = STX, etx = ETX, dle = DLE, sum = 2;
	HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &command, 1, 1000);
	if (haveData != 0)
	{
		uint8_t *temp;
		temp = Float2Char(data);
		for (uint8_t i = 0; i< sizeof(temp); i++)
		{
			if (*(temp+i) == ETX)
			{
				HAL_UART_Transmit(&hlpuart1, &dle, 1, 1000);
				HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
			}
			else
			{
				HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
			}
		}
		sum = 6;
	}
	HAL_UART_Transmit(&hlpuart1, &sum, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
}

void TransmitDataOrigin (float realtime, float measure)
{
	uint8_t stx = STX, etx = ETX, dle = DLE, head = Data, sum = 10;
	uint8_t *temp;
	HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &head, 1, 1000);
	temp = Float2Char(realtime);
	for (uint8_t i = 0; i< sizeof(temp); i++)
	{
		if (*(temp+i) == ETX)
		{
			HAL_UART_Transmit(&hlpuart1, &dle, 1, 1000);
			HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
		}
		else
		{
			HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
		}
	}
	temp = Float2Char(measure);
	for (uint8_t i = 0; i< sizeof(temp); i++)
	{
		if (*(temp+i) == ETX)
		{
			HAL_UART_Transmit(&hlpuart1, &dle, 1, 1000);
			HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
		}
		else
		{
			HAL_UART_Transmit(&hlpuart1, temp+i, 1, 1000);
		}
	}
	HAL_UART_Transmit(&hlpuart1, &sum, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
}

void TransmitData (float realtime, float measure, float pwm)
{
	uint8_t stx = STX, etx = ETX, head = Data;
	uint8_t *temp;
	HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &head, 1, 1000);
	temp = Float2Char(realtime);
	HAL_UART_Transmit(&hlpuart1, temp, 4, 1000);
	temp = Float2Char(measure);
	HAL_UART_Transmit(&hlpuart1, temp, 4, 1000);
	temp = Float2Char(pwm);
	HAL_UART_Transmit(&hlpuart1, temp, 4, 1000);
	HAL_UART_Transmit(&hlpuart1, &txindex, 1, 1000);
	HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
}

/**
 * Gửi gói tin (gồm realtime và measure) và chạy nghi thức handshake: Nếu nhận Nak thì gửi lại gói tin. Nếu quá timeout thì gửi tiếp gói
 * tin khác, bất kể gói tin vừa gửi có được GUI nhận hay không (Do dữ liệu thời gian thực nên dữ liệu cũ không
 * còn ý nghĩa)
 *
 * @para buffer: buffer chứa dữ liệu để truyền xuống uart
 * @para realtime: thời gian hiện tại
 * @para measure: giá trị đo được hiện tại
 *
 * @return: Truyền thành công hay không
 */
uint8_t TransmitAndHandshake (float *buffer, float realtime, float measure, float pwm)
{
	if (txnak || !txack)
	{
		TransmitData(*buffer, *(buffer+1), *(buffer + 2));
		return 0;
	}
	else
	{
		txindex++;
		if (txindex > 100) txindex = 0;
		TransmitData(realtime, measure, pwm);
		*buffer = realtime;
		*(buffer+1) = measure;
		*(buffer+2) = pwm;
		txack = 0;

		__HAL_TIM_SET_COUNTER(&htim6, 0);
		HAL_TIM_Base_Start_IT(&htim6);
		return 1;
	}
}

/**
 * Nhận gói tin, chạy nghi thức handshake: Nếu nhận đầy đủ STX, ETX và checksum của gói tin
 * bằng với số byte nhận được thì handshake thành công
 *
 * @para buffer: buffer chứa dữ liệu đọc được từ uart
 * @para instruction: xuất câu lệnh và tham số từ gói tin và truyền ra ngoài
 *
 * @return: đã hoàn thành handshake hay chưa (0: chưa, 1: hoàn thành)
 */
uint8_t ReceiveAndHandshakeOrigin (uint8_t *buffer, uint8_t *instruction)
{
//	Thực hiện đọc gói tin nếu như nhận được STX và hiện tại chưa xử lý gói tin nào, bằng cách bật cờ haveFrame
	if ((*buffer == STX) && !haveFrame)
	{
		haveFrame = 1;
		rxindex = 0;
		return 0;
	}

//	Nếu chưa có gói tin (chưa có haveFrame) thì không xử lý
	if (!haveFrame)
	{
		return 0;
	}
//	Nếu có gói tin thì xử lý gói tin bằng cách đọc lần lượt từng byte
	else
	{
		static uint8_t rxi = 0;
		rxbuffer[rxi++] = *buffer;

//		Nếu byte đọc được là ETX (kết thúc gói tin)
		if (*buffer == ETX)
		{
//			Kiểm tra byte trước đó có phải là DLE không. Nếu đúng thì xóa byte DLE đi, thay bằng ETX
//			và tiếp tục đọc gói tin
			if (rxbytefield[rxindex-1] == DLE)
			{
				rxbytefield[rxindex-1] = *buffer;
			}
//			Nếu không có DLE, thực hiện quá trình kết thúc xử lý gói tin
			else
			{
//				Kiểm tra checksum: so sánh số byte nhận được với byte checksum trong gói tin. Nếu đúng thì
//				truyền ngược lại gói tin vừa nhận để ACK với GUI, và xóa hết các cờ haveFrame, ... Copy
//				các dữ liệu nhận được và biến instruction và xuất ra ngoài. Return 1 xác nhận hoàn thành handshake
				if (rxindex == rxbytefield[rxindex-1])
				{
					uint8_t stx = STX, etx = ETX;
					HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
					HAL_UART_Transmit(&hlpuart1, rxbuffer, rxi-1, 1000);
					HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
					for (uint8_t i = 0; i<= (rxindex-2); i++)
						*(instruction+i) = rxbytefield[i];
					rxindex = 0;
					haveFrame = 0;
					rxi = 0;
					return 1;
				}
//				Nếu không đúng checksum thì gửi NAK để yêu cầu gửi lại gói tin
				else
				{
					uint8_t nak = Nak, stx = STX, etx = ETX;
					HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
					HAL_UART_Transmit(&hlpuart1, &nak, 1, 1000);
					HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
					rxindex = 0;
					haveFrame = 0;
					return 0;
				}
			}
		}
//		Liên tục nhận các byte trong gói tin
		else
		{
			rxbytefield[rxindex++] = *buffer;
//			Nếu nhận quá nhiều byte thì hủy gói tin đang xử lý, đợi nhận gói tin khác
			if (rxindex > 6)
			{
				rxindex = 0;
				haveFrame = 0;
			}
		}
	}
	return 0;
}

uint8_t ReceiveAndHandshake (uint8_t *buffer, uint8_t *instruction)
{
	if (*buffer != STX)
	{
		rxnak = 1;
	}

	if (*(buffer + 6) != ETX)
	{
		rxnak = 1;
	}

	if (!rxnak)
	{
		for (uint8_t i = 0; i<= 4; i++)
			*(instruction+i) = *(buffer+i+1);
		HAL_UART_Transmit(&hlpuart1, buffer, 7, 1000);
		return 1;
	}
	else
	{
		uint8_t nak = Nak, stx = STX, etx = ETX;
		uint8_t dummy[4] = {0,0,0,0};
		HAL_UART_Transmit(&hlpuart1, &stx, 1, 1000);
		HAL_UART_Transmit(&hlpuart1, &nak, 1, 1000);
		HAL_UART_Transmit(&hlpuart1, dummy, 4, 1000);
		HAL_UART_Transmit(&hlpuart1, &etx, 1, 1000);
		rxnak = 0;
		return 0;
	}
}

uint8_t ReceiveAck (uint8_t *buffer)
{
	if (*buffer != STX)
	{
		txnak = 1;
	}

	if (*(buffer + 6) != ETX)
	{
		txnak = 1;
	}

	if (!txnak)
	{
		if (*(buffer + 1) == Ack)
		{
			if (*(buffer+5) == txindex)
			{
				txack = 1;

				HAL_TIM_Base_Stop_IT(&htim6);
				return 1;
			}
			else
			{
				txnak = 1;
				return 2;
			}
		}
		else if (*(buffer + 1) == Nak)
		{
			txnak = 1;
			return 2;
		}
	}
	//else
	{
		return 0;
	}
}

float ReceiveDataofHeader (uint8_t *rxbuffer)
{
	float data;
	uint8_t temp[4];
	*temp = *(rxbuffer + 1);
	*(temp + 1) = *(rxbuffer + 2);
	*(temp + 2) = *(rxbuffer + 3);
	*(temp + 3) = *(rxbuffer + 4);
	data = Char2Float(temp);
	return data;
}

void EchoReceived (uint8_t *rxbuff)
{
	HAL_UART_Transmit(&hlpuart1, rxbuff, 4, 1000);
	//while (!__HAL_UART_GET_FLAG(&hlpuart1,UART_FLAG_TC));
}




/* Data Converting ----------------------*/

/**
 * Chuyển số nguyên thành số thực. Nhận vào mảng 4 byte số nguyên và trả về số thực
 *
 */
float Char2Float (uint8_t *charNum)
{
	float floatNum = 0;
	float* pFloat;
    long temp = 0;
    temp = *charNum;
    temp = (temp << 8) + *(charNum+1);
    temp = (temp << 8) + *(charNum+2);
    temp = (temp << 8) + *(charNum+3);
    pFloat = (float*)&temp;
    floatNum = *pFloat;
    return floatNum;
}

/**
 * Chuyển số thực thành số nguyên. Nhận vào số thực và trả về 4 byte số nguyên
 */
uint8_t* Float2Char (float floatNum)
{
	static uint8_t charNum[4];
	long *pLong;
	pLong = (long*)&floatNum;
	charNum[0] = *pLong>>24;
	charNum[1] = *pLong>>16;
	charNum[2] = *pLong>>8;
	charNum[3] = *pLong;
	return charNum;
}

