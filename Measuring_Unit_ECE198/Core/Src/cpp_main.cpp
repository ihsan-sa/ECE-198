/*
 * cpp_main.cpp
 *
 *  Created on: Nov 25, 2024
 *      Author: ihsan
 */


//
//#include "main.h"
//#include "uart.hpp"
//
//
//int cpp_main(){
//
//
//	UART_Connection uart1(UART_TX_Pin, GPIOA, UART_RX_Pin, GPIOB, 10, 0b11011110);
//	char message[10] = {'H', 'e','l','l','o','\0'};
//	uart1.send_message(message);
//	printf("Message sent\n\r");
//
//	while(1){
//		HAL_GPIO_WritePin(GPIOB, UART_TX_Pin, GPIO_PIN_SET);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, UART_TX_Pin, GPIO_PIN_RESET);
//		HAL_Delay(1000);
//	}
//
//	return 0;
//}
