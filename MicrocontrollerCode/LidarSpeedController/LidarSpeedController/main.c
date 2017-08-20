/*
 * LidarSpeedController.c
 *
 * Created: 2017-07-15 8:28:40 PM
 * Author : funmi
 */

#include <avr/io.h>
#include <stddef.h>
#include "uart.h"
#include <util/delay.h>
#include "string.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define TO_MILLISEC(x) (x)*(F_CPU/1000)
//#define DEBUG

/* circular buffer ADT*/
char* addToBuffer(){
  while (buffer_mutex == 0){}
  buffer_mutex = 0;
  char* buffer = outgoing_frame_contents[buffer_index];
  current_buffer_size++;
  if (current_buffer_size >= MAX_BUFFER_SIZE){
    current_buffer_size = MAX_BUFFER_SIZE;
  }
  buffer_index = (buffer_index + 1) %  MAX_BUFFER_SIZE;
  buffer_mutex = 1;
  return buffer;
}

char* removeFromBuffer(){
  while (buffer_mutex == 0){}
  buffer_mutex = 0;
  if (current_buffer_size == 0)
    return NULL;
  char* buffer;
  if (current_buffer_size != MAX_BUFFER_SIZE)
    buffer = outgoing_frame_contents[mod((buffer_index - current_buffer_size), MAX_BUFFER_SIZE)];
  else
    buffer = outgoing_frame_contents[buffer_index];

  current_buffer_size--;
  buffer_index--;
  if (buffer_index == -1)
    buffer_index = MAX_BUFFER_SIZE - 1;
  buffer_mutex = 1;
  return buffer;
}

int mod(int a, int b){
  if(b < 0) //you can check for b == 0 separately and do what you want
  return mod(a, -b);
  int ret = a % b;
  if(ret < 0)
  ret+=b;
  return ret;
}

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
  UART_Transmit(' ');
  printNumber((int)rpm, 3);
  UART_Transmit('\r');
  UART_Transmit('\n');
}

void SendLidarFrame(char* frame){
  //repackage the lidar frame and send it over UART
  //replace the start character with an l
  frame[0] = 'l';
  //extract the speed
  rpm = (float)(((uint16_t)frame[3] << 8) | (uint16_t)frame[2]) / 64.0;
  //remove the speed from the data
  memmove(frame+2, frame+4, INCOMING_FRAME_SIZE-4);
  //replace the checksum data
  frame[OUTGOING_FRAME_SIZE-1] = OutgoingChecksum(frame , OUTGOING_FRAME_SIZE-1);
  
  //Negotiate control of the Bluetooth TX
  uint32_t count = 0;
  while((PIND & (1 << PIND4)) && (count++ < TO_MILLISEC(.5))){}
  if (count > TO_MILLISEC(0.5))
    return;

  //signal to the other device
  PORTD |= (1 << PIND3);
  PrintCharArray(frame, OUTGOING_FRAME_SIZE, 1);
  _delay_us(200);
  PORTD &= !(1 << PIND3);
}

ISR(USART_RX_vect){
  static int frame_size_counter = 0;
  static char* buffer_reference;
  cli();
  char data = UDR0;
  if (data == 0xFA){
    frame_size_counter = 0;
  }
  if (frame_size_counter < INCOMING_FRAME_SIZE){
    incoming_frame_contents[frame_size_counter++] = data;
    if (frame_size_counter == INCOMING_FRAME_SIZE){
      if (GetChecksum(incoming_frame_contents) == (((uint16_t)incoming_frame_contents[INCOMING_FRAME_SIZE-1] << 8) | (uint16_t)incoming_frame_contents[INCOMING_FRAME_SIZE-2])){
        //TODO Remove the following line
        strncpy(incoming_frame_contents, "0123456789ABCDEFGHIJKL", INCOMING_FRAME_SIZE);
        buffer_reference = addToBuffer();
        strncpy(buffer_reference, incoming_frame_contents, INCOMING_FRAME_SIZE);
      }
    }
  }
  sei();
}

int main(void){
  current_buffer_size = 0;
  buffer_index = 0;
  buffer_mutex = 1;
  memset(outgoing_frame_contents, 0, MAX_BUFFER_SIZE * INCOMING_FRAME_SIZE);
  rpm = 0;
  int set = 225;
  //set up PD3 and PD4 as connection to the main arduino one as an output and the other as an input
  DDRD |= (1 << DDD3);//PD3 as output
  PORTD &= !(1 << PIND3);// turn off PD3

  char frame[INCOMING_FRAME_SIZE];
  char* buffer_reference;
  UART_Init(MYUBRR);
  sei();
  PrintCharArray("Started", 7, 1);
  
  SetupPwm();
  SetSpeed(set);
  while (1){
    while (current_buffer_size){
      buffer_reference = removeFromBuffer();
      cli();
      strncpy(frame, buffer_reference, INCOMING_FRAME_SIZE);
      sei();
      SendLidarFrame(frame);
    }
     //TODO after every encoder speed update correct the lidar motor output with a PID
    #ifdef PRINT_SPEED
    PrintSpeed();
    #endif
  }
}