/*
 * uart_sigma.h
 *
 *  Created on: Nov 25, 2024
 *      Author: ihsan
 */

#ifndef INC_UART_SIGMA_H_
#define INC_UART_SIGMA_H_

#include "main.h"
#include "stm32f4xx_hal.h"


unsigned int baud_rate = 5;


void UART_init(){
	HAL_GPIO_WritePin(GPIOB,  UART_TX_Pin, GPIO_PIN_SET);
}

void UART_send_byte(int data){

	long double time_period = ((long double)1000)/((long double)baud_rate);

	//hold down for one time period
	HAL_GPIO_WritePin(GPIOB,  UART_TX_Pin, GPIO_PIN_RESET);
	HAL_Delay(time_period);
	HAL_GPIO_WritePin(GPIOB,  UART_TX_Pin, GPIO_PIN_SET);


	int mask = 1;

	for(int bit = 0; bit < 8; bit++){
		if((data & mask) == 0){
			HAL_GPIO_WritePin(GPIOB, UART_TX_Pin, GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOB, UART_TX_Pin, GPIO_PIN_SET);
		}

		mask <<= 1;

		HAL_Delay(time_period);

	}

	//set high again
	HAL_GPIO_WritePin(GPIOB, UART_TX_Pin, GPIO_PIN_SET);
}

int UART_read_byte(){

	printf("Waiting for start\n\r");
	long double time_period = ((long double)1000)/((long double)baud_rate);

	while(HAL_GPIO_ReadPin( GPIOA,  UART_RX_Pin) == 1);
	HAL_Delay(time_period);

	//delay an additional half period for better accuracy

	HAL_Delay(time_period/2);

	printf("Receiving\n\r");
	//now read each bit

	int received_byte = 0;

	for(int bit = 0; bit < 8; bit++){
		received_byte |= HAL_GPIO_ReadPin( GPIOA,  UART_RX_Pin);
		printf("%d", HAL_GPIO_ReadPin( GPIOA,  UART_RX_Pin));
		received_byte <<= 1;
		HAL_Delay(time_period);
	}

	return received_byte;


}



#endif /* INC_UART_SIGMA_H_ */
