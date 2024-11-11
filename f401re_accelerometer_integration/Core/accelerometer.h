/*
 * accelerometer.h
 *
 *  Created on: Oct 8, 2024
 *      Author: ihsan
 *
 *
 *  General info:
 *
 *  SPI Interface only for the MPU-6000
 *
 *
 *  CS is active LOW
 *  Data transmission is MSB then LSB
 *  Data latched on rising edge of CLK
 *  Data should be transitioned on falling edge of the clock
 *  Max CLK 1MHx
 *
 *  for addr:
 *  first byte is R/W - Read (1) or Write (0)
 *
 *
 *Timing::
 *
 *Default:: CS HIGH, CLK HIGH
 *
 *CS LOW
 *CLK LOW
 *
 *WRITE MOSI BIT
 *HOLD
 *CLK HIGH
 *
 *
 *
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

int send_byte(int byte){

}

#endif /* ACCELEROMETER_H_ */
