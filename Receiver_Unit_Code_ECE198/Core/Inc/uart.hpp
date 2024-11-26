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

#ifndef INC_UART_HPP_
#define INC_UART_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"
#include <iostream>


//declaration
class UART_Connection;

//definition
typedef class UART_Connection{
	std::uint16_t tx_pin_;
	GPIO_TypeDef* tx_port_;
	std::uint16_t rx_pin_;
	GPIO_TypeDef* rx_port_;
	unsigned int baud_rate_;
	unsigned int message_end_char_;

public:
	UART_Connection(std::uint16_t tx_pin,
					GPIO_TypeDef* tx_port,
					std::uint16_t rx_pin,
					GPIO_TypeDef* rx_port,
					unsigned int baud_rate,
					unsigned int message_end_char);

	int read();
	int listen();
	char *read_message(char *result);
	int *read_data(int *result);
	void send_data(int *data);
	void send_message(char message[]);
	void send_byte(int byte);

}UART_Connection;


//implementation

UART_Connection::UART_Connection(std::uint16_t tx_pin,
								GPIO_TypeDef* tx_port,
								std::uint16_t rx_pin,
								GPIO_TypeDef* rx_port,
								unsigned int baud_rate,
								unsigned int message_end_char){
	this->tx_pin_ = tx_pin;
	this->tx_port_ = tx_port;
	this->rx_pin_ = rx_pin;
	this->rx_port_ = rx_port;
	this->baud_rate_ = baud_rate;
	this->message_end_char_ = message_end_char;

}

int UART_Connection::read(){

	long double time_period = 1.0/this->baud_rate_; //time for each bit
	long double delay = time_period/2; //time for half a bit
	int result{0};
	HAL_Delay(time_period*1000);//wait one time period
	HAL_Delay(delay*1000); //wait for the middle of the bit, then read it.

	//Read MSB first
	for(int i = 0; i < 8; i++){
		int received_bit = HAL_GPIO_ReadPin(this->rx_port_, this->rx_pin_);
		result |= received_bit;
		result <<= 1;
		HAL_Delay(time_period*1000);
	}

	return result;

}
int UART_Connection::listen(){
	while(HAL_GPIO_ReadPin(this->rx_port_, this->rx_pin_) != 1);
	return read();
}
char *UART_Connection::read_message(char *result){
	int i = 0;
	int received{};
	do{
		received = this->listen();
		result[i] = (char)received;
		i++;
	}while(received != (int)this->message_end_char_);
	result[i-1] = '\0'; //set last char to the null char.

	return result;
}
int *UART_Connection::read_data(int *result){
	int i = 0;
	int received{};
	do{
		received = this->listen();
		result[i] = received;
		i++;
	}while(received != (int)this->message_end_char_);
	result[i-1] = 0;
	return result;
}

void UART_Connection::send_byte(int byte){

	long double time_period = 1.0/this->baud_rate_;

	//set tx low first
	HAL_GPIO_WritePin(this->tx_port_, this->tx_pin_, GPIO_PIN_RESET);
	HAL_Delay(time_period*1000);

	//Send MSB first
	int mask = 0b10000000;
	for(int i{0}; i < 8; i++){
		HAL_GPIO_WritePin(this->tx_port_, this->tx_pin_, ((byte & mask) == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET );
		mask >>= 1;
		HAL_Delay(time_period*1000);

	}
	//set tx to high again
	HAL_GPIO_WritePin(this->tx_port_, this->tx_pin_, GPIO_PIN_SET);
}

void UART_Connection::send_message(char message[]){

	long double time_period = 1.0/this->baud_rate_;
	int i{0};

	while(message[i] != '\0'){
		this->send_byte((int)message[i]);
		HAL_Delay(time_period*2*1000);
	}
	this->send_byte(this->message_end_char_);
	HAL_Delay(time_period*1000);
	HAL_GPIO_WritePin(this->tx_port_, this->tx_pin_, GPIO_PIN_SET); //set the TX pin to 1 again
}

void UART_Connection::send_data(int *data){

	long double time_period = 1.0/this->baud_rate_;
	int i{0};

	while(data[i] != (int)this->message_end_char_){
		this->send_byte(data[i]);
		HAL_Delay(time_period*1000*2);
	}
	this->send_byte(this->message_end_char_);
	HAL_Delay(time_period*1000);
	HAL_GPIO_WritePin(this->tx_port_, this->tx_pin_, GPIO_PIN_SET); //set the TX pin to 1 again
}





#endif /* INC_UART_HPP_ */
