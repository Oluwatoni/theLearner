/*
 * initialization.h
 *
 * Created: 08/04/2016 6:08:25 PM
 *  Author: Toni
 */ 

#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

#define F_CPU 16000000
#define FOSC F_CPU
#define BAUD 19200
#define MYUBRR FOSC/16/BAUD-1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/cpufunc.h>
#include "TWI_slave.h"
#include <stdint-gcc.h>
#include <string.h>

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20
#define USART_BUFFER_SIZE 80 //NMEA sentences cannot be longer than 80 characters 
#define MESSEAGE_TYPE_SIZE 6 //all _SIZE contain extra byte for checksum
#define LATITUDE_SIZE 13
#define LONGITUDE_SIZE 14
#define TIME_SIZE 10
#define SPEED_SIZE 6
#define COURSE_SIZE 6
#define ALTITUDE_SIZE 7
#define TEMP_SIZE 50
#define CHECKSUM_SIZE 2

char received_data[USART_BUFFER_SIZE];
static char* first_char = received_data;
static uint8_t usart_index = 0;
char time[TIME_SIZE];
char latitude[LATITUDE_SIZE];
char longitude[LONGITUDE_SIZE];
char altitude[ALTITUDE_SIZE];
char speed[SPEED_SIZE];
char course[COURSE_SIZE];
uint8_t data_is_ready;

void USART_Init( unsigned int ubrr);
void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );
void extractData(char* message_index);
void printCharArray(char array[], uint8_t size);
uint8_t fillCharArray(char array[], char* pointer, uint8_t size, uint8_t no_of_commas);
uint8_t compareCharArray(char a[], char b[], uint8_t size);

#endif /* INITIALIZATION_H_ */