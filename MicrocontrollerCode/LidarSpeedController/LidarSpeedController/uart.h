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

#define INCOMING_FRAME_SIZE 22
#define OUTGOING_FRAME_SIZE 19

#define MAX_BUFFER_SIZE 3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint-gcc.h>
#include <math.h>

volatile char incoming_frame_contents[INCOMING_FRAME_SIZE];
volatile float rpm;

/* circular buffer ADT*/
volatile char outgoing_frame_contents[MAX_BUFFER_SIZE][INCOMING_FRAME_SIZE];
volatile int current_buffer_size;
volatile int buffer_index;
volatile int buffer_mutex;

void UART_Init( unsigned int);
void UART_Transmit( unsigned char data );
unsigned char UART_Receive( void );
void PrintCharArray(char array[], uint8_t size, uint8_t newline);
void printNumber(uint16_t value, uint8_t digits);
uint16_t GetChecksum(char* data_frame);

#endif /* UART_H_ */