/*
 * uartCommunication.h
 *
 *  Created on: Nov 26, 2024
 *      Author: dhyey
 */

#ifndef SRC_UARTCOMMUNICATION_H_
#define SRC_UARTCOMMUNICATION_H_

void sendMsg(GPIO_TypeDef* gpioTypeTransmit, int transmitPin, int8_t message, int baudDelay, int bufferDelay){
	// start bit
	HAL_GPIO_WritePin(gpioTypeTransmit, transmitPin, 0);
	HAL_Delay(baudDelay);

	// send message
	for (int i=0; i<8; i++){
		HAL_GPIO_WritePin(gpioTypeTransmit, transmitPin, !!(message & (1 << 7-i)) );
//
//		if ( !!(message & (1 << 15-i)) ){
//			printf("1\r\n");
//		}
//		else{
//			printf("0\r\n");
//		}

		HAL_Delay(baudDelay);
	}
//	printf("\n");
	// go back to default high
	HAL_GPIO_WritePin(gpioTypeTransmit, transmitPin, 1);
	HAL_Delay(bufferDelay);
}


uint8_t readMsg(GPIO_TypeDef* gpioTypeRead, int readPin, int baudDelay){
	uint8_t got_msg = 0;
	printf("waiting for start bit\r\n");
	while(HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
		//wait for start bit
	}

	printf("received start bit\r\n");
	HAL_Delay(baudDelay+10); // let start bit end, delay a bit
	for (int i=0; i<8; i++){
		got_msg |= HAL_GPIO_ReadPin(gpioTypeRead, readPin) << 7-i;
//		if (HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//			printf("1");
//		}
//		else{
//			printf("0");
//		}
//		printf("\r\n");
	    HAL_Delay(baudDelay);
	}

	return got_msg;
}


#endif /* SRC_UARTCOMMUNICATION_H_ */
