/*
 * LidarSpeedController.c
 *
 * Created: 2017-07-15 8:28:40 PM
 * Author : funmi
 */

#include <avr/io.h>
#include <stddef.h>
#include "uart.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

//#define DEBUG

void SetupPwm(){
  DDRD |= (1 << DDD6);
  // PD6 is now an output
  OCR0A = 0;
  // set PWM for 0% duty cycle
  TCCR0A |= (1 << COM0A1);
  // set none-inverting mode
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  // set fast PWM Mode
  TCCR0B |= (1 << CS01);
  // set prescaler to 8 and starts PWM
}

void SetSpeed(uint8_t duty_cycle){
  OCR0A = duty_cycle;
}

char OutgoingChecksum(char* data, size_t size){
  uint32_t total = 0;
  for (uint8_t i = 0; i < size; i++)
    total += data[i];
  return (total % 255);
}

void PrintSpeed(){
  printNumber((incoming_frame_contents[1]- 0xA0), 3);
  UART_Transmit('  ');
  printNumber((int)rpm, 3);
  UART_Transmit('\r');
  UART_Transmit('\n');
}

void SendLidarFrame(){
  //Negotiate control of the Bluetooth TX

  uint32_t count = 0;
  while((PIND & (1 << PIND4)) && (count++ < LIDAR_FRAME_TIMEOUT)){}
  if (count == LIDAR_FRAME_TIMEOUT+1)  {
    PrintCharArray("lid Time out!", 13, 1);
    return;
  }

  //signal to the other device
  PORTD |= (1 << PIND3);
  _NOP();
  _NOP();
  _NOP();
  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);

  //PrintCharArray(outgoing_frame_contents, OUTGOING_FRAME_SIZE, 0);
  PrintCharArray("ABCDEFGHIJKLMNOPQRS", OUTGOING_FRAME_SIZE, 1);

  UCSR0B = (1<<RXEN0)|(1<<RXCIE0);
  sei();
  _NOP();
  _NOP();
  _NOP();
  PORTD &= !(1 << PIND3);
}

int main(void){
  data_ready = 0;
  rpm = 0;
  int set = 225;
  //set the TXD as an input when not sending
  //set up PD3 and PD4 as connection to the main arduino one as an output and the other as an input
  DDRD |= (1 << DDD3);//PD3 as output
  PORTD &= !(1 << PIND3);// turn off PD3

  sei();
  UART_Init(MYUBRR);
  SetupPwm();
  SetSpeed(set);
  while (1){
    if (data_ready){
      cli();
      memcpy(outgoing_frame_contents, incoming_frame_contents, OUTGOING_FRAME_SIZE);
      sei();
      data_ready = 0;

#ifdef PRINT_SPEED
      PrintSpeed();
#endif
      //repackage the lidar frame and send it over UART
      //replace the start character with an l
      outgoing_frame_contents[0] = 'l';
      //replace the checksum data
      outgoing_frame_contents[OUTGOING_FRAME_SIZE-1] = OutgoingChecksum(outgoing_frame_contents, OUTGOING_FRAME_SIZE-1);

      SendLidarFrame();
      //TODO after every encoder speed update correct the lidar motor output with a PID
      rpm = (float)(((uint16_t)outgoing_frame_contents[3] << 8) | (uint16_t)outgoing_frame_contents[2]) / 64.0;
    }
  }
}