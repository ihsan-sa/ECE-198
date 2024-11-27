/*
 * mpuDataCollection.h
 *
 *  Created on: Nov 26, 2024
 *      Author: dhyey
 */

#ifndef SRC_MPUDATACOLLECTION_H_
#define SRC_MPUDATACOLLECTION_H_

void writeMPU(GPIO_TypeDef* gpioTypeSDA, int sdaPin, GPIO_TypeDef* gpioTypeSCL,
			  int sclPin, GPIO_TypeDef* gpioTypeRead, int readPin, int deviceAddress, int writeAddress,
			  int message, int delayVal) {

//	printf("writing to device\r\n");
//	HAL_Delay(1000);
//	printf("test hal delay\r\n");

	// start condition:
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);


	// send device address
	for (int i=0; i<7; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(deviceAddress & (1 << 6-i)) );
	    HAL_Delay(delayVal);

//	    if (HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//	    	printf("1\r\n");
//	    }
//	    else{
//	    	printf("0\r\n");
//	    }

	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	    HAL_Delay(delayVal);
	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	    HAL_Delay(delayVal);
	}


	// write is logic low
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

//	if (HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		    	printf("1\r\n");
//		    }
//		    else{
//		    	printf("0\r\n");
//		    }

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

//	if (HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		    	printf("1\r\n");
//		    }
//		    else{
//		    	printf("0\r\n");
//		    }

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("device address received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0); // reset SDA line to low after the device lets go of the SDA line
	HAL_Delay(delayVal);


	// send internal register address
	for (int i=0; i<8; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(writeAddress & (1 << 7-i)) );
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	    HAL_Delay(delayVal);
	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	    HAL_Delay(delayVal);
	}


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("register address received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0); // reset SDA line to low after the device lets go of the SDA line
	HAL_Delay(delayVal);


	// send data to register, for power register of MPU, 0 means it keeps it always awake
	for (int i=0; i<8; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(message & (1 << 7-i)) );
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	    HAL_Delay(delayVal);
	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	    HAL_Delay(delayVal);
	}


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("message received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0); // reset SDA line to low after the device lets go of the SDA line
	HAL_Delay(delayVal);



	// stop condition
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_Delay(delayVal);
}


int readMPU(GPIO_TypeDef* gpioTypeSDA, int sdaPin, GPIO_TypeDef* gpioTypeSCL, int sclPin,
			GPIO_TypeDef* gpioTypeRead, int readPin, int deviceAddress, int readAddress, int delayVal) {

//	printf("reading from device\r\n");

	int storeData = 0;
	// start condition:
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);


	// send device address
	for (int i=0; i<7; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(deviceAddress & (1 << 6-i)) );
		HAL_Delay(delayVal);

		HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
		HAL_Delay(delayVal);
		HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
		HAL_Delay(delayVal);

		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
		HAL_Delay(delayVal);
	}


	// write is logic low
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("device address received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0); // reset SDA line to low after the device lets go of the SDA line
	HAL_Delay(delayVal);



	// send internal register address
	for (int i=0; i<8; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(readAddress & (1 << 7-i)) );
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	    HAL_Delay(delayVal);
	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	    HAL_Delay(delayVal);
	}


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("register address received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0); // reset SDA line to low after the device lets go of the SDA line
	HAL_Delay(delayVal);



	// start condition again
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);


	// send device address
	for (int i=0; i<7; i++){
		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, !!(deviceAddress & (1 << 6-i)) );
		HAL_Delay(delayVal);

		HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
		HAL_Delay(delayVal);
		HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
		HAL_Delay(delayVal);

		HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
		HAL_Delay(delayVal);
	}


	// read is logic high
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);


	// read acknowledge bit
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);  // set SDA line to default high state
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);

	if (!HAL_GPIO_ReadPin(gpioTypeRead, readPin)){
//		printf("register address to READ received\r\n");
	}
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);


	// keep SDA line default high before device pulls line and input stream of bits comes in
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_Delay(delayVal);


	// Receive register data
	for (int i=0; i<8; i++){
		HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	    HAL_Delay(delayVal);

	    storeData |= (HAL_GPIO_ReadPin(gpioTypeRead, readPin)) * (1 << 7-i); // add bit to acclZ
	    HAL_Delay(delayVal);

	    HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0); // now after this clock pulse, the sda line should change to corresponding value
	    HAL_Delay(delayVal); // pause it a bit longer at 0 just in case
	}


	// send nack signal
	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 1);
	HAL_Delay(delayVal);

	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 0);
	HAL_Delay(delayVal);
//	printf("sent NACK signal\r\n");

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);


	//stop condition
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sdaPin, 1);
	HAL_Delay(delayVal);


	return storeData;
}

#endif /* SRC_MPUDATACOLLECTION_H_ */
