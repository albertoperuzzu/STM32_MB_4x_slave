#include <MB_comm.h>
#include "main.h"

bool buildFrame;
bool frameReady;
uint8_t MBFunc;
uint8_t rxData;
uint8_t globalCounter;
char ModbusRx[BUFFER_SIZE];
char localModbusRx[BUFFER_SIZE];
char ModbusTx[BUFFER_SIZE];
uint16_t MB_Holding_reg[BUFFER_SIZE];

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // Check if the Interrupt comes from USART1
    {
    	// Handle the byte received
    	Handle_MB_RX();
    	// Set the UART to be prepared to receive another byte
		HAL_UART_Receive_IT(&huart1 , &rxData , 1);
		// Restart Timer TIM2
		HAL_TIM_Base_Stop_IT(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start_IT(&htim2);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
    	// Set the UART to be prepared to receive another byte
        HAL_GPIO_WritePin(GPIOA, PIN_DE_Pin, GPIO_PIN_RESET);
    }
}

void Handle_MB_RX(void)
{
	// The Master has addressed the correct Slave ID, therefore the bytes are collected in the buffer
	if(buildFrame)
	{
		ModbusRx[globalCounter] = rxData;
		globalCounter++;
		if(globalCounter == 2)
		{
			// Define the Function Code of the request
			if(rxData == READ_H_REG || rxData == WRITE_SINGLE_REG || rxData == WRITE_MULTI_REG)
			{
				MBFunc = rxData;
			}
			else
			{
				globalCounter = 0;
				buildFrame = 0;
			}
		}
	}
	// If counter  is 0 and the ID is correct, the master is addressing the slave
	// Preparing the buffer to be filled
	else if(globalCounter == 0 && rxData == MB_ID)
	{
		ModbusRx[0] = rxData;
		globalCounter++;
		buildFrame = true;
	}
	// The byte received has no interest for the slave
	else
	{
		globalCounter = 0;
		buildFrame = false;
		MBFunc = 0;
	}
	// The frame could be completed
	if(globalCounter >= 8)
	{
		// If the Function Code is not 0x10, the frame is completed if it contains 8 bytes
		if(MBFunc != WRITE_MULTI_REG)
		{
			frameReady = true;
		}
		// If the Function Code is 0x10, define the length of the message and keep receiving
		else
		{
			uint8_t len = ModbusRx[5] * 2 + 9;
			if(globalCounter >= len)
			{
				frameReady = true;
			}
		}
	}
}

void Handle_MB_TX(void)
{
	char localModbusRx[BUFFER_SIZE];
	bool CRCGood;
	uint8_t localCounter;

	// Copy the received buffer and global parameters into local ones to reset the RX
	memcpy(localModbusRx, ModbusRx, globalCounter + 1);
	localCounter = globalCounter;
	globalCounter = 0;
	buildFrame = false;
	frameReady = false;
	memset(ModbusRx, 0, localCounter);
	memset(ModbusTx, 0, localCounter);
	// Check of the received CRC
	CRCGood = Check_CRC(&localModbusRx[0], localCounter);
	if(CRCGood)
	{
		// Define the way to process the received frame
		switch(MBFunc)
		{
			case 3:
				build_03_packet(&localModbusRx[0]);
				break;
			case 6:
				build_06_packet(&localModbusRx[0], localCounter);
				break;
			case 16:
				build_16_packet(&localModbusRx[0], localCounter);
				break;
		}
	}
}

bool Check_CRC(char *localModbusRx, uint8_t localCounter)
{
	uint16_t CRCValue;
	uint16_t CRCrx;

	CRCValue = MB_crc16(localModbusRx, localCounter - 2);
	CRCrx = (localModbusRx[localCounter -1] << 8) | (localModbusRx[localCounter - 2]);
	if(CRCrx == CRCValue)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void build_03_packet(char *msg)
{
	uint8_t i, m = 0;
	uint16_t reg_addr = 0;
	uint16_t reg_num = 0;
	uint16_t crc_val;

	// Analyze the reading request
	reg_addr = (msg[2] << 8) | (msg[3]);
	reg_num = (msg[4] << 8) | (msg[5]);
	// Prepare the TX buffer
	ModbusTx[0] = msg[0];
	ModbusTx[1] = msg[1];
	ModbusTx[2] = (reg_num * 2);
	for(i = 0; i < reg_num * 2; i += 2)
	{
		ModbusTx[3 + i] = (uint8_t)(MB_Holding_reg[reg_addr + m] >> 8);
		ModbusTx[4 + i] = (uint8_t)(MB_Holding_reg[reg_addr + m] & 0x00FF);
		m++;
	}
	crc_val = MB_crc16(ModbusTx, 3 + (reg_num * 2 ));
	ModbusTx[4 + (reg_num * 2 )] = (crc_val >> 8);
	ModbusTx[3 + (reg_num * 2 )] = (crc_val & 0x00FF);
	// Send the response
	txEXEC(ModbusTx, 5 + (reg_num * 2 ));
}

void build_06_packet(char *msg, uint8_t Lenght)
{
	uint16_t reg_addr, reg_val;

	// Analyze the writing request
	reg_addr = (msg[2] << 8) | (msg[3]);
	reg_val = (msg[4] << 8) | (msg[5]);
	// Update the value
	MB_Holding_reg[reg_addr] = reg_val;
	// Send the response
	txEXEC(msg, Lenght);
	// Execute custom actions
	WriteSingleReg(reg_addr, reg_val);
}

void build_16_packet(char *msg, uint8_t Lenght)
{
	uint16_t reg_addr, reg_num, crc_val;
	uint8_t i,m = 0;

	// Analyze the writing request
	reg_addr = (msg[2] << 8) | (msg[3]);
	reg_num = (msg[4] << 8) | (msg[5]);
	// Update the values
	for(i = 0; i < reg_num; i++)
	{
		MB_Holding_reg[reg_addr + i] = (uint16_t)((uint16_t)msg[7 + m] << 8) | (msg[8 + m]);
		m += 2;
	}
	// Prepare Response
	memcpy(ModbusTx, msg, 6);
	crc_val = MB_crc16(ModbusTx, 6);
	ModbusTx[6] = (crc_val & 0x00FF);
	ModbusTx[7] = (crc_val >> 8);
	// Send the response
	txEXEC(ModbusTx, 8);
	// Execute custom actions
	WriteMultipleReg(reg_addr, reg_num, msg);
}

void txEXEC(char *msg, uint8_t len)
{
	// Set the DE PIN to Transmit
	HAL_GPIO_WritePin(GPIOA, PIN_DE_Pin, GPIO_PIN_SET);
	// Custom Delay
	HAL_Delay(10);
	// Set the UART to be prepared to receive another byte
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)msg, len);
}

void WriteSingleReg(uint16_t reg_addr, uint16_t reg_val)
{
	switch(reg_addr)
	{
	case 0:
		// Call custom function
		break;
	case 1:
		// Call custom function
		break;
	default:
		// Call custom function
		break;
	}
}

void WriteMultipleReg(uint16_t reg_addr, uint16_t reg_num, char *msg)
{
	uint8_t i = 0;

	for(i = 0; i < reg_num; i++)
	{
		// Call custom function
	}
}

uint16_t MB_crc16(char *buf, uint8_t len )
{
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			xor = crc & 0x01;
			crc >>= 1;
			crc ^= table[xor];
		}
	}
	return crc;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // Check if the Interrupt comes from TIM2
    {
        // Timeout expired: reset parameters and buffers
    	globalCounter = 0;
    	buildFrame = false;
    	frameReady = false;
    	MBFunc = 0;
    	memset(ModbusRx, 0, BUFFER_SIZE);
    	memset(ModbusTx, 0, BUFFER_SIZE);
    	// Stop the timer to avoid other interrupts
        HAL_TIM_Base_Stop_IT(&htim2);
    }
}
