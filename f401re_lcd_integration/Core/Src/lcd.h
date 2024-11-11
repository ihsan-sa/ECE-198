/*
 * lcd.h
 *
 *  Created on: Oct 8, 2024
 *      Author: Ihsan Salari
 *
 *      Connect and configure STM32 pins as follows:
Label        (Port):

LCD_D0         (PB4)
LCD_D1         (PB5)
LCD_D2         (PC0)
LCD_D3         (PC1)
LCD_D4         (PB0)
LCD_D5         (PA4)
LCD_D6         (PA1)
LCD_D7         (PA0)
LCD_RS         (PA9)
LCD_RW         (PA8)
LCD_E          (PB10)
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_


#include "main.h"
#include "stdio.h"


  void reset_lcd_bits(){
  	HAL_GPIO_WritePin(GPIOB, LCD_D0_Pin, 0);
  	HAL_GPIO_WritePin(GPIOB, LCD_D1_Pin, 0);
  	HAL_GPIO_WritePin(GPIOC, LCD_D2_Pin, 0);
  	HAL_GPIO_WritePin(GPIOC, LCD_D3_Pin, 0);
  	HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin, 0);
  	HAL_GPIO_WritePin(GPIOA, LCD_D5_Pin, 0);
  	HAL_GPIO_WritePin(GPIOA, LCD_D6_Pin, 0);
  	HAL_GPIO_WritePin(GPIOA, LCD_D7_Pin, 0);
  	HAL_GPIO_WritePin(GPIOA, LCD_RW_Pin, 0);
  	HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, 0);
  	//Enable pin
  	HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, 0);

  }
  void send_lcd_instruction(int instruction, int rw, int rs){

  	printf("Sending instruction: %d  0x%x  %c  0b",instruction ,instruction,(char)instruction);
  	//set everything to 0
  	reset_lcd_bits();


  	int mask = 1;
  	int result = 0;
  	int parsed_instruction[8];

  	for(int bit = 0; bit<8;bit++){

  		mask = 1 << bit;
  		result = instruction & mask;
  		parsed_instruction[bit] = result >> bit;

  		switch (result){
  		case 0b00000001:
  			HAL_GPIO_WritePin(GPIOB, LCD_D0_Pin, 1); break;
  		case 0b00000010:
  			HAL_GPIO_WritePin(GPIOB, LCD_D1_Pin, 1); break;
  		case 0b00000100:
  			HAL_GPIO_WritePin(GPIOC, LCD_D2_Pin, 1); break;
  		case 0b00001000:
  			HAL_GPIO_WritePin(GPIOC, LCD_D3_Pin, 1); break;
  		case 0b00010000:
  			HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin, 1); break;
  		case 0b00100000:
  			HAL_GPIO_WritePin(GPIOA, LCD_D5_Pin, 1); break;
  		case 0b01000000:
  			HAL_GPIO_WritePin(GPIOA, LCD_D6_Pin, 1); break;
  		case 0b10000000:
  			HAL_GPIO_WritePin(GPIOA, LCD_D7_Pin, 1); break;
  		}

  	}
  	for(int i = 0; i<8;i++) printf("%d", parsed_instruction[7-i]);

  	if(rw == 1)	{
  		HAL_GPIO_WritePin(GPIOA, LCD_RW_Pin, 1);
  		printf("  R/W: 1  ");
  	}else{
  		printf("  R/W: 0  ");
  	}
  	if(rs == 1) {
  		HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, 1);
  		printf("RS: 1");
  	}else{
  		printf("RS: 0");
  	}
  	printf("\r\n");

  //	toggle enable_bit

  	HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, 1);
  	HAL_Delay(10);
  	HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, 0);

  // set everything to zero again
  	reset_lcd_bits();
  	HAL_Delay(10);

  }

#define set_mode 0b00111000;
#define display_config 0b00001110;
#define entry_mode_set 0b00000110;

  void lcd_print_chars(char chars[], int delay){
	  int len = 0;
	  while(chars[len] != '\0')len++;

	  for(int c = 0; c < len; c++){
		  send_lcd_instruction((int)chars[c],0,1);
		  HAL_Delay(10+delay);
	  }
  }

  void lcd_clear(){send_lcd_instruction(0b00000001,0,0);}


  void lcd_init(){
  	HAL_Delay(10);
  	send_lcd_instruction(0b00111000, 0, 0); //set config
  	send_lcd_instruction(0b00001111, 0,0); //Display on;cursor; blink off
  	send_lcd_instruction(0b00000110,0,0);
  	lcd_clear();

  }


#endif /* SRC_LCD_H_ */
