#ifndef INC_MB_COMM_H_
#define INC_MB_COMM_H_

#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

// ModBus ID of the slave
#define MB_ID  1  
// Size of the buffer, keep a multiple of 16 for simplicity
#define BUFFER_SIZE  32  
// MB Code for reading Holding Registers
#define READ_H_REG  0x03 
// MB Code for writing a single Holding Register
#define WRITE_SINGLE_REG  0x06  
// MB Code for writing multiple Holding Registers
#define WRITE_MULTI_REG  0x010  

extern uint16_t MB_Holding_reg[BUFFER_SIZE];  // Array of the global Holding Registers
extern bool buildFrame;  // Flag for correct ID addressing
extern bool frameReady;  // Flag for frame completion
extern uint8_t MBFunc;  // Define the function of the received message
extern uint8_t rxData;  // The last byte received from UART
extern uint8_t globalCounter;  // Number of bytes consecutively received at the same address
extern char ModbusRx[BUFFER_SIZE];  // Global buffer of bytes received
extern char localModbusRx[BUFFER_SIZE];  // Local buffer of bytes received
extern char ModbusTx[BUFFER_SIZE];  // Global buffer of bytes to transmit

void Handle_MB_RX(void);
void Handle_MB_TX(void);
bool Check_CRC(char *localModbusRx, uint8_t localCounter);
void build_03_packet(char *msg);
void build_06_packet(char *msg, uint8_t Lenght);
void build_16_packet(char *msg, uint8_t Lenght);
void txEXEC(char *msg, uint8_t len);
void WriteSingleReg(uint16_t reg_addr, uint16_t reg_val);
void WriteMultipleReg(uint16_t reg_addr, uint16_t reg_num, char *msg);
uint16_t MB_crc16(char *buffer, uint8_t lenght );

#endif /* INC_MB_COMM_H_ */
