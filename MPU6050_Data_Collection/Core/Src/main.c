/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
/* Support printf over UART */
(void) HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFFU);
 return ch;
}


void writeMPU(GPIO_TypeDef* gpioTypeSDA, int sdaPin, GPIO_TypeDef* gpioTypeSCL,
			  int sclPin, GPIO_TypeDef* gpioTypeRead, int readPin, int deviceAddress, int writeAddress,
			  int message, int delayVal) {

	printf("writing to device\r\n");
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
		printf("device address received\r\n");
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
		printf("register address received\r\n");
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
		printf("message received\r\n");
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

	printf("reading from device\r\n");

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
		printf("device address received\r\n");
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
		printf("register address received\r\n");
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
		printf("register address to READ received\r\n");
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
	printf("sent NACK signal\r\n");

	HAL_GPIO_WritePin(gpioTypeSDA, sdaPin, 0);
	HAL_Delay(delayVal);


	//stop condition
	HAL_GPIO_WritePin(gpioTypeSCL, sclPin, 1);
	HAL_Delay(delayVal);
	HAL_GPIO_WritePin(gpioTypeSCL, sdaPin, 1);
	HAL_Delay(delayVal);


	return storeData;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  int mpuDeviceAddress = 0x68; // first 7 bits of i2c data
  int acclZAxisRegisterBits15_8 = 0x3F; // 0x3f, 0x40
  int acclZAxisRegisterBits7_0 = 0x40;
  int powerRegister = 0x6B;


  int sda = GPIO_PIN_8; // D7 --> PA_8
  #define sdaType GPIOA

  int scl = GPIO_PIN_9; //D8 --> PA_9
  #define sclType GPIOA

  int read = GPIO_PIN_5; //D4 --> PB_5
  #define readType GPIOB

  int delayTime = 1; // 1 ms

  int16_t acclZ = 0;
  float acclMS2 = 0;
  // wake up MPU
  writeMPU(sdaType, sda, sclType, scl, readType, read, mpuDeviceAddress, powerRegister, 0, delayTime);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	acclZ = 0;
	acclZ |= ((readMPU(sdaType, sda, sclType, scl, readType, read, mpuDeviceAddress, acclZAxisRegisterBits15_8, delayTime)) << 8);
	acclZ |= readMPU(sdaType, sda, sclType, scl, readType, read, mpuDeviceAddress, acclZAxisRegisterBits7_0, delayTime);
	acclMS2 = (float)(acclZ) / 16000.0 * 9.8;

	printf("\n");
	printf("%d\r\n", acclZ);

	printf("\n");
	printf("%d", (int) acclMS2);
	printf(".");
	printf("%d\r\n", ( (int)(acclMS2 * 10.0) ) % 10);

	printf("\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
//	HAL_Delay(4000);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
//	HAL_Delay(1000);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
