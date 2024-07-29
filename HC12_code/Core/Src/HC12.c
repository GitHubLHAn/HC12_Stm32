#include<HC12.h>

/*
NOTE: 
	
*/
/**********************************************************************************************************************************/

/*Extern the UART*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

extern uint32_t time_tick;

/**********************************************************************************************************************************/
/*Declare struct variable*/
HC12_t vHC12;

uint16_t fl_var = 0;

Result_t result_Cmd;

/**********************************************************************************************************************************/
/*Function*/

/*______________________________________________________________________________*/

void change_baudrate(UART_HandleTypeDef *phuart, uint32_t baudrate){
	HAL_UART_DeInit(phuart);
	phuart->Init.BaudRate = baudrate;
	HAL_UART_Init(phuart);
}

void send_AT(UART_HandleTypeDef *phuart){
	uint32_t last_baud= phuart->Init.BaudRate;
	change_baudrate(phuart, 9600);
	
	uint8_t AT_cmd[2] = "AT";

	HAL_UART_Transmit(&huart1, AT_cmd, sizeof(AT_cmd),10);
	
	change_baudrate(phuart, last_baud);
}

void set_baudrate(UART_HandleTypeDef *phuart, uint32_t baudrate_HC12){		// set baudrate for HC-12

	uint32_t last_baud= phuart->Init.BaudRate;
	change_baudrate(phuart, 9600);
	
	unsigned char AT_cmd[10];
	sprintf((char*)AT_cmd, "AT+B%d", baudrate_HC12);
	
	HAL_UART_Transmit(&huart1, AT_cmd, sizeof(AT_cmd),10);

	
	change_baudrate(phuart, last_baud);
}

void set_channel(UART_HandleTypeDef *phuart, uint8_t channel_HC12){		// set baudrate for HC-12

	uint32_t last_baud= phuart->Init.BaudRate;
	unsigned char AT_cmd[7];
	change_baudrate(phuart, 9600);
	
	if(channel_HC12<100){
		if(channel_HC12 < 10)
			sprintf((char*)AT_cmd, "AT+C00%d", channel_HC12);
		else
			sprintf((char*)AT_cmd, "AT+C0%d", channel_HC12);
	}
	else
		sprintf((char*)AT_cmd, "AT+C%d", channel_HC12);

	HAL_UART_Transmit(&huart1, AT_cmd, sizeof(AT_cmd),10);

	change_baudrate(phuart, last_baud);
}

void set_transMode(UART_HandleTypeDef *phuart, transMode_t mode){		// set baudrate for HC-12

	uint32_t last_baud= phuart->Init.BaudRate;
	unsigned char AT_cmd[6];
	change_baudrate(phuart, 9600);
	
	if(mode == FU1)
		sprintf((char*)AT_cmd, "AT+FU1");
	else if(mode == FU2)
		sprintf((char*)AT_cmd, "AT+FU2");
	else		// default
		sprintf((char*)AT_cmd, "AT+FU3");

	HAL_UART_Transmit(&huart1, AT_cmd, sizeof(AT_cmd),10);
	
	change_baudrate(phuart, last_baud);
}

void HC12_Config(UART_HandleTypeDef *phuart, uint32_t baudrate_HC12, uint8_t channel_HC12, transMode_t mode){		// set baudrate for HC-12
		send_AT(&huart1);
		HAL_Delay(50);
		set_baudrate(&huart1, baudrate_HC12);
		HAL_Delay(50);
		set_channel(&huart1, channel_HC12);
		HAL_Delay(50);
		set_transMode(&huart1, mode);
}

void start_rx(UART_HandleTypeDef *phuart, uint8_t *rx_buffer, uint16_t timeout){
	HAL_UARTEx_ReceiveToIdle_DMA(phuart, rx_buffer, timeout);
}

void trans_data(UART_HandleTypeDef *phuart, uint8_t *tx_buffer, uint16_t size){
	//HAL_UART_Transmit(phuart, tx_buffer, sizeof(tx_buffer), timeout);
	HAL_UART_Transmit_DMA(phuart, tx_buffer, LENGHT_TX);
}
/*______________________________________________________________________________*/
	/*Checksum with CRC8*/
	unsigned char Check_CRC8(unsigned char *data, uint16_t data_length) {
			uint16_t length = data_length - 1;
			unsigned char crc = 0;
			uint16_t i, j;
		
			for (i = 0; i < length; i++){
				crc ^= data[i];
				for (j = 0; j < 8; j++){
					if (crc & 0x80)
							crc = (crc << 1) ^ 0x07;
					else 
							crc <<= 1;
					crc &= 0xFF;
				}
			}
			return crc;
	}
/*______________________________________________________________________________*/
	void HC12_Init(HC12_t *pHC){
		pHC->flag_tx = false;
		pHC->flag_rx = false;
		memset(pHC->tx_buffer, 0x00, LENGHT_TX);
		memset(pHC->rx_buffer, 0x00, LENGHT_TX);
		pHC->tx_buffer[0] = 0xFF;	// header byte
		pHC->modeRun = false; pHC->isDone = false; pHC->isReceived = false; pHC->isSent = false;
		pHC->start_time = 0;	pHC->current_time = 0;
		pHC->CS_byte = 0;
	}
/*______________________________________________________________________________*/
	/*Tao frame truyen data*/
	void Gen_data_TX(uint8_t *tx_array, uint8_t robotID, uint8_t mode, uint8_t number){
		tx_array[0] = 0xFF;
		tx_array[1]= (robotID>>8) & 0xFF;			// byte 1 ID robot
		tx_array[2]= robotID & 0xFF ;					// byte 0 ID robot
		tx_array[3] = mode;										// mode
		if(mode == DI_TIEN || mode == DI_LUI)		// so o dich chuyen
			tx_array[4] = number;
		else		
			tx_array[4] = 0;
		static uint16_t var = 0;
		tx_array[4] = var++;
		for(uint8_t j = 5; j<20; j++)
			tx_array[j] = rand() % 255; 
		tx_array[19]= Check_CRC8(tx_array, LENGHT_TX);	
//		for(uint8_t j = 1; j<20; j++)
//			tx_array[j] = j;
	}
/*______________________________________________________________________________*/	
	void Handle_HC12_RX(unsigned char *dataRX, unsigned char *dataTX){
		if(dataRX[0] == 0xFF){			// check header
			if( ( (dataRX[1] << 8) | dataRX[2]) == 1){		// check ID
				if(dataRX[3] == dataTX[3]){			// robot received cmd
					vHC12.isReceived = 0;			

					
					if(dataRX[4] == dataTX[4]){		// robot finish the cmd		
						vHC12.CS_byte = Check_CRC8(vHC12.rx_buffer, LENGHT_RX);
						if(vHC12.CS_byte == dataRX[LENGHT_RX-1]){
							vHC12.modeRun = 0;
							memset(vHC12.tx_buffer, 0x00, LENGHT_TX);		// clear the tx_buffer
							
							result_Cmd = RbFinishCmd;	// to Debug
						}
						else
							result_Cmd = Wrong_Checksum;
					}
					else{
						vHC12.CS_byte = Check_CRC8(vHC12.rx_buffer, LENGHT_RX);
						if(vHC12.CS_byte == dataRX[LENGHT_RX-1])						
							result_Cmd = RbReceivedCmd;	// to Debug
						else
							result_Cmd = Wrong_Checksum;
					}	
				} 	
			}
			result_Cmd = WrongID;	// wrong ID	
		}
		else
			result_Cmd = WrongHeader;		// wrong header
		
		memset(&vHC12, 0x00, LENGHT_RX); 		// clear the rx_buffer
	}

void sendCMD(unsigned char *tx_array, uint8_t robotID, uint8_t mode){
	if(vHC12.isDone == 0){			// sending cmd successfully???
		if(vHC12.isSent == 0){		/// not sent yet
			vHC12.isSent = 1;
			/*Make the frame transmit*/
			tx_array[0] = 0xFF;
			tx_array[1]= (robotID>>8) & 0xFF;			// byte 1 ID robot
			tx_array[2]= robotID & 0xFF ;					// byte 0 ID robot
			tx_array[3] = mode;			// mode
			if(mode == DI_TIEN || mode == DI_LUI)		// so o dich chuyen
				tx_array[4] = vHC12.num_carpet;
			else		
				tx_array[4] = 0;
			tx_array[LENGHT_TX-1]= Check_CRC8(tx_array, LENGHT_TX);
			/*Transmit commend data*/
			trans_data(&huart1, vHC12.tx_buffer, LENGHT_TX);
			/*Calculate the start time of sending cmd*/
			vHC12.start_time = time_tick*65536 + __HAL_TIM_GetCounter(&htim1); 
		}
		else{
			/*Calculate the current time*/
			vHC12.current_time = time_tick*65536 + __HAL_TIM_GetCounter(&htim1);
			/*Send again the cmd if do not receiced the response after 1ms*/
			if((vHC12.current_time - vHC12.start_time) > 1000){
				if(vHC12.isReceived == 1){
					vHC12.isDone = 1;			// stop request processing
					vHC12.start_time = 0;
					vHC12.current_time = 0;
				}
				else	
					vHC12.isSent = 0;		// send again
			}
		}
	}
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	/* Prevent unused argument(s) compilation warning */
//	UNUSED(huart);
//	UNUSED(Size);
//	if(huart->Instance == huart3.Instance)
//	{
//		fl_var++;
//		if(Size == 20){
//			vHC12.flag_rx++;// = 1;
//		}
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, vHC12.rx_buffer, 50);
//	}
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if(huart->Instance == huart3.Instance)
//	{
//		fl_var++;
//		if(Size == 20){
//			vHC12.flag_rx++;// = 1;
//		}
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, vHC12.rx_buffer, 50);
//	}
