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
#define MESSEAGE_TYPE_SIZE 5
#define LATITUDE_SIZE 12 //all _SIZE contain extra byte for checksum except checksum and message_type
#define LONGITUDE_SIZE 13
#define TIME_SIZE 9
#define SPEED_SIZE 5
#define COURSE_SIZE 5
#define ALTITUDE_SIZE 6
#define TEMP_SIZE 50
#define GPS_CHECKSUM_SIZE 2
#define NUM_OF_MESSAGES 6

char received_data[USART_BUFFER_SIZE];
static char* first_char = received_data;
static uint8_t usart_index = 0;
char time[TIME_SIZE];
char latitude[LATITUDE_SIZE];
char longitude[LONGITUDE_SSIZE];
char altitude[ALTITUDE_SIZE];
char speed[SPEED_SIZE];
char course[COURSE_SIZE];
static char I2C_checksum_array[NUM_OF_MESSAGES] = {'d','u','m','m','i','e'};
uint8_t data_is_ready;

typedef enum data_type
{
  LAT,LONG,ALT,SPD,CRSE,TIME,NONE
}gps_data_type_t;  

void USART_Init( unsigned int);
void USART_Transmit( unsigned char );
unsigned char USART_Receive( void );
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char );
void extractData(char*);
void printCharArray(char* , uint8_t);
uint8_t fillCharArray(char* , char*, uint8_t, uint8_t, gps_data_type_t);
uint8_t compareCharArray(char* , char* , uint8_t );
void prepare_TWI_data (gps_data_type_t, char*, uint8_t);

#endif /* INITIALIZATION_H_ */