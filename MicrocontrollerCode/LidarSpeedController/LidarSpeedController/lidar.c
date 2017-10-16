/*
 * lidar.c
 *
 * Created: 2017-08-21 11:33:42 PM
 *  Author: funmi
 */

#include "Lidar.h"

/************************************************************************/
/*                          Circular Queue                              */
/************************************************************************/
int mod(int a, int b){
  if(b < 0) //you can check for b == 0 separately and do what you want
    return mod(a, -b);
  int ret = a % b;
  if(ret < 0)
    ret+=b;
  return ret;
}

char* enqueue(){
  char* buffer;

  while (queue_mutex == 0){}
  queue_mutex = 0;
  buffer = outgoing_frame_contents[queue_index];
  current_queue_size++;
  if (current_queue_size >= MAX_QUEUE_SIZE)
    current_queue_size = MAX_QUEUE_SIZE;
  queue_index = (queue_index + 1) %  MAX_QUEUE_SIZE;
  lidar_data_ready = 1;
  queue_mutex = 1;
  return buffer;
}

char* dequeue(){
  char* buffer;

  while (queue_mutex == 0){}
  queue_mutex = 0;
  if (current_queue_size == 0)
    return NULL;
  if (current_queue_size != MAX_QUEUE_SIZE)
    buffer = outgoing_frame_contents[mod((queue_index - current_queue_size), MAX_QUEUE_SIZE)];
  else
    buffer = outgoing_frame_contents[queue_index];
  current_queue_size--;
  if (current_queue_size == 0)
    lidar_data_ready = 0;
  queue_index--;
  if (queue_index == -1)
    queue_index = MAX_QUEUE_SIZE - 1;
  queue_mutex = 1;
  return buffer;
}

/************************************************************************/
/*                 Lidar Motor Speed Control                            */
/************************************************************************/
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

void PrintSpeed(){
  PrintNumber((incoming_frame_contents[1]- 0xA0), 3);
  UART_Transmit(' ');
  PrintNumber((int)rpm, 3);
  UART_Transmit('\r');
  UART_Transmit('\n');
}
/************************************************************************/
/*                    Lidar Communication                               */
/************************************************************************/
void lidarInit(){
  current_queue_size = 0;
  queue_index = 0;
  queue_mutex = 1;
  rpm = 0;
  lidar_data_ready = 0;

  //set up PD3 and PD4 as connection to the main arduino one as an output and the other as an input
  DDRD |= (1 << DDD3);//PD3 as output
  PORTD &= !(1 << PIND3);// turn off PD3
  
  memset(outgoing_frame_contents, 0, MAX_QUEUE_SIZE * INCOMING_FRAME_SIZE);
  
  UART_Init(MYUBRR);
  SetupPwm();
}

char OutgoingChecksum(char* data, size_t size){
  uint32_t total = 0;
  for (uint8_t i = 0; i < size; i++)
    total += data[i];
  return (total % 255);
}

void SendLidarFrame(){
  static char* buffer_reference;
  static char frame[MAX_QUEUE_SIZE][INCOMING_FRAME_SIZE];
  static uint32_t count;
  int num_of_frames = current_queue_size;

  for (int i = 0; i < num_of_frames; i++){
    buffer_reference = dequeue();

    {
      cli();
      strncpy(frame[i], buffer_reference, INCOMING_FRAME_SIZE);
      sei();
    }

    //replace the start character with an l
    frame[i][0] = 'l';
    //extract the speed
    if (i == (num_of_frames-1))
      rpm = (float)(((uint16_t)frame[i][3] << 8) | (uint16_t)frame[i][2]) / 64.0;
    //remove the speed from the data
    memmove(frame[i]+2, frame[i]+4, INCOMING_FRAME_SIZE-4);
    //replace the checksum data
    frame[i][OUTGOING_FRAME_SIZE-1] = OutgoingChecksum(frame[i] , OUTGOING_FRAME_SIZE-1);
  }
  
  //Negotiate control of the Bluetooth TX
  count = 0;
  while((PIND & (1 << PIND4)) && (count++ < TO_MILLISEC(.5))){}
  if (count > TO_MILLISEC(0.5))
    return;

  //signal to the other device
  PORTD |= (1 << PIND3);
  for(int i = 0; i < num_of_frames; i++)
    PrintCharArray(frame[i], OUTGOING_FRAME_SIZE, 1);
  _delay_us(200);
  PORTD &= !(1 << PIND3);

  buffer_reference = NULL;
  memset(frame, 0, MAX_QUEUE_SIZE * INCOMING_FRAME_SIZE);
}

uint16_t NeatoChecksum(char* data_frame){
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
      if (NeatoChecksum(incoming_frame_contents) == (((uint16_t)incoming_frame_contents[INCOMING_FRAME_SIZE-1] << 8) 
          | (uint16_t)incoming_frame_contents[INCOMING_FRAME_SIZE-2])){
        buffer_reference = enqueue();
        strncpy(buffer_reference, incoming_frame_contents, INCOMING_FRAME_SIZE);
      }
    }
  }
  sei();
}
