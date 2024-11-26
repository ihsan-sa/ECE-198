/*
 * uart.hpp
 *
 *  Created on: Nov 20, 2024
 *      Author: ihsan
 *
 *      UART Class
 *
 *      D10 - TX - Yellow
 *      D11 - RX - Orange
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "stm32f4xx_hal.h"


unsigned int baud_rate_ = 1000;
unsigned int message_end_char_ = 0b11011110;

	//declaration




//implementation

//UART_TX_Pin, GPIOA, UART_RX_Pin, GPIOB, 10, 0b11011110

//void UART_Connection(std::uint16_t tx_pin,
//								GPIO_TypeDef* tx_port,
//								std::uint16_t rx_pin,
//								GPIO_TypeDef* rx_port,
//								unsigned int baud_rate,
//								unsigned int message_end_char){
//	UART_TX_Pin = tx_pin;
//	GPIOB = tx_port;
//	UART_RX_Pin = rx_pin;
//	GPIOA = rx_port;
//	baud_rate_ = baud_rate;
//	message_end_char_ = message_end_char;
//
//}

int read(){

	long double time_period = 1.0/ baud_rate_; //time for each bit
	long double delay = time_period/2; //time for half a bit
	int result = 0;
	HAL_Delay(time_period*1000);//wait one time period
	HAL_Delay(delay*1000); //wait for the middle of the bit, then read it.

	//Read MSB first
	for(int i = 0; i < 8; i++){
		int received_bit = HAL_GPIO_ReadPin( GPIOA,  UART_RX_Pin);
		result |= received_bit;
		result <<= 1;
		HAL_Delay(time_period*1000);
	}

	return result;

}
int listen(){
	while(HAL_GPIO_ReadPin( GPIOA,  UART_RX_Pin) != 1);
	return read();
}
char *read_message(char *result){
	int i = 0;
	int received;
	do{
		received =  listen();
		result[i] = (char)received;
		i++;
	}while(received != (int) message_end_char_);
	result[i-1] = '\0'; //set last char to the null char.

	return result;
}
int *read_data(int *result){
	int i = 0;
	int received;
	do{
		received =  listen();
		result[i] = received;
		i++;
	}while(received != (int) message_end_char_);
	result[i-1] = 0;
	return result;
}

void send_byte(int byte){

	long double time_period = 1.0/ baud_rate_;

	//set tx low first
	HAL_GPIO_WritePin( GPIOB,  UART_TX_Pin, GPIO_PIN_RESET);
	HAL_Delay(time_period*1000);

	//Send MSB first
	int mask = 0b10000000;
	for(int i = 0; i < 8; i++){

		if((byte & mask) == 1){
			HAL_GPIO_WritePin(GPIOB,  UART_TX_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB,  UART_TX_Pin, GPIO_PIN_RESET);
		}
		mask >>= 1;
		HAL_Delay(time_period*1000);

	}
	//set tx to high again
	HAL_GPIO_WritePin( GPIOB,  UART_TX_Pin, GPIO_PIN_SET);
}

void send_message(char message[]){

	long double time_period = 1.0/ baud_rate_;
	int i = 0;

	while(message[i] != '\0'){
		 send_byte((int)message[i]);
		HAL_Delay(time_period*2*1000);
	}
	send_byte( message_end_char_);
	HAL_Delay(time_period*1000);
	HAL_GPIO_WritePin( GPIOB,  UART_TX_Pin, GPIO_PIN_SET); //set the TX pin to 1 again
}

void send_data(int *data){

	long double time_period = 1.0/ baud_rate_;
	int i = 0;

	while(data[i] != (int) message_end_char_){
		 send_byte(data[i]);
		HAL_Delay(time_period*1000*2);
	}
	 send_byte( message_end_char_);
	HAL_Delay(time_period*1000);
	HAL_GPIO_WritePin( GPIOB,  UART_TX_Pin, GPIO_PIN_SET); //set the TX pin to 1 again
}





#endif /* INC_UART_HPP_ */
