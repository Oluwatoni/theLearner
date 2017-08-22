/*
 * uart.h
 *
 * Created: 2017-05-07 7:29:37 PM
 *  Author: funmi
 * just a UART library to speed up debugging on the atmega328p dw it was tested!
 */

#ifndef UART_H_
#define UART_H_

#define F_CPU 20000000UL
#define FOSC F_CPU
#define BAUD 57600UL//the speed is doubled in the init function
#define MYUBRR FOSC/16UL/BAUD-1UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint-gcc.h>
#include <math.h>

void UART_Init(unsigned int);
void UART_Transmit(unsigned char);
unsigned char UART_Receive(void);

void PrintCharArray(char*, uint8_t, uint8_t);
void PrintNumber(uint16_t, uint8_t);

#endif /* UART_H_ */