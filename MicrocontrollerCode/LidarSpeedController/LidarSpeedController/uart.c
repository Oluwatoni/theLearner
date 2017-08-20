/*
 * uart.c
 *
 * Created: 2017-05-07 7:29:53 PM
 *  Author: funmi
 */
#include "uart.h"

void UART_Init( unsigned int ubrr){
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;

  /*Double the transmission speed*/
  UCSR0A |= 1 << U2X0;

  /*Enable receiver and transmitter and RX interrupt */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = (3<<UCSZ00);
}

void UART_Transmit( unsigned char data ){
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

void printNumber(uint16_t value, uint8_t digits){
  uint16_t remainder;
  //value = (uint16_t) value;
  while(digits--){
    remainder = (int)(value / pow(10,digits));
    if (remainder <= 9){
      UART_Transmit(remainder + '0');
      value -= remainder * pow(10,digits);
    }
  }
}

unsigned char UART_Receive( void ){
  /* Wait for data to be received */
  while ( !(UCSR0A & (1<<RXC0)) );
  /* Get and return received data from buffer */
  return UDR0;
}

void PrintCharArray(char array[], uint8_t size, uint8_t newline){
  for (uint8_t i = 0;i < size; i++)
    UART_Transmit(*(array++));
  if (newline){
    //UART_Transmit('\r');
    UART_Transmit('\n');
  }
  //waits for the data to finish being transmitted
  while(!(UCSR0A & ((1 << TXC0)))){}
}

uint16_t GetChecksum(char* data_frame){
  //Compute and return the checksum as an int
  //data -- list of 20 bytes (as ints), in the order they arrived in.
  //group the data by word, little-endian
  uint16_t data_list[10];
  for (int i = 0; i < 10; i++)
    data_list[i] = ( data_frame[2*i] + (data_frame[2*i+1]<<8) );

  //# compute the checksum on 32 bits
  uint32_t chk32 = 0;
  for (int i = 0; i < 10; i++)
    chk32 = (chk32 << 1) + data_list[i];

  //return a value wrapped around on 15bits, and truncated to still fit into 15 bits
  uint16_t checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); //# wrap around to fit into 15 bits
  checksum = checksum & 0x7FFF; //# truncate to 15 bits
  return checksum;
}