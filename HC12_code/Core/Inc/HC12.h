/*
 * USER.h
 * Created on: 21-Mar-2024
 * Author: Le Huu An
 */

#ifndef HC12_H_
#define HC12_H_

#include<main.h>
/*Include the type of stm32*/
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"  
#include "stdlib.h"
#include "stdbool.h"

/*DEFINE*/
#define LENGHT_TX 20
#define LENGHT_RX 25


#define QUAY_TRAI 	0x10
#define QUAY_PHAI 	0x11
#define DI_TIEN		0x12
#define QUAY_LAI	0x13
#define DI_LUI	0x14
#define CHECK_RES	0xB1
#define MOVE_DONE	0xD0
			

/************************************************************************************/
/*DECLARE STRUCT*/
	typedef enum{
	FU1, 
	FU2, 
	FU3 // is default
	}transMode_t;
	
	typedef struct{
		unsigned char tx_buffer[LENGHT_TX];
		unsigned char rx_buffer[LENGHT_RX];
		bool flag_tx, flag_rx;
		uint8_t modeRun;
		bool isDone, isReceived, isSent;
		uint32_t start_time, current_time;
		uint16_t num_carpet;
		unsigned char CS_byte;
	}HC12_t;
	
	typedef enum{
		WrongID, WrongHeader, Wrong_Checksum,
		RbReceivedCmd, RbFinishCmd
	
	}Result_t;
	
/*DECLARE FUNCTION*/
void change_baudrate(UART_HandleTypeDef *phuart, uint32_t baudrate);
void send_AT(UART_HandleTypeDef *phuart);
void set_baudrate(UART_HandleTypeDef *phuart, uint32_t baudrate_HC12);
void set_channel(UART_HandleTypeDef *phuart, uint8_t channel_HC12);
void HC12_Config(UART_HandleTypeDef *phuart, uint32_t baudrate_HC12, uint8_t channel_HC12, transMode_t mode);
	
void start_rx(UART_HandleTypeDef *phuart, uint8_t *rx_buffer, uint16_t timeout);
void trans_data(UART_HandleTypeDef *phuart, uint8_t *tx_buffer, uint16_t size);
void HC12_Init(HC12_t *pHC);

uint8_t Check_CRC8(uint8_t *data, uint16_t data_length);
	
void HC12_Init(HC12_t *pHC);
void Handle_HC12_RX(unsigned char *dataRX, unsigned char *dataTX);
	
void Gen_data_TX(unsigned char *tx_array, uint8_t robotID, uint8_t mode, uint8_t number);
void sendCMD(unsigned char *tx_array, uint8_t robotID, uint8_t mode);



/************************************************************************************/
#endif /* HC12_H */


