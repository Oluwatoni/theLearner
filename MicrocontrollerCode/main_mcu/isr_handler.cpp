/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
*/  
#include "Arduino.h"
#include "main_mcu.hpp"

char input_string[40];         // a string to hold incoming data
uint8_t input_string_index = 0;
boolean input_string_complete = false;  // whether the string is complete

//handles RC msgs
void extractData(char * msg, uint8_t msg_size) {
  int16_t state = 0, temp_steering, temp_throttle, temp_stop, checksum_size = 0, checksum;
  String raw_steering = "", raw_throttle = "", raw_stop = "", raw_checksum = "";
  char temp[15];

  for (int i = 0; i < msg_size; i++) {
    if (*msg == ',')
      state++;
    switch (state) {
      case 0:
        //r before the first comma
        temp[checksum_size++] = *msg;
        break;
      case 1:
        //Steering
        if (*msg != ',')
          raw_steering.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 2:
        //Throttle
        if (*msg != ',')
          raw_throttle.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 3:
        if (*msg != ',')
          raw_stop.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 4:
        if (*msg != '\n' && *msg != ',')
          raw_checksum.concat(*msg);
        break;
    }
    msg++;
  }
  //temp[checksum_size++] = ',';

  if ((uint8_t)generateChecksum(temp, checksum_size) == raw_checksum.toInt()){
#ifdef DEBUG
    Serial.println(raw_steering);
    Serial.println(raw_throttle);
    Serial.println(raw_stop);
#endif

    temp_steering = raw_steering.toInt();
    temp_throttle = raw_throttle.toInt();
    temp_stop = raw_stop.toInt();
  }
#ifdef DEBUG
  else{
    for (int i = 0; i < checksum_size; i++){
      Serial.print(temp[i]);
    }
    Serial.println();
    Serial.println(raw_checksum);
    Serial.println((uint8_t)generateChecksum(temp, checksum_size));
  }
#endif
}

//using Cristian's algorithm to update the arduino's time
void recieveTime(char * msg, uint8_t msg_size) {
  uint8_t state = 0, checksum_size = 0, checksum;
  uint32_t int_second, int_microsecond;
  String raw_second = "", raw_microsecond = "", raw_checksum = "";
  char temp[40];

  for (int i = 0; i < msg_size; i++){
    if (*msg == ',')
      state++;
    switch (state){
      case 0:
        //t before the first comma
        temp[checksum_size++] = *msg;
        break;
      case 1:
        if (*msg != ',')
          raw_second.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 2:
        if (*msg != ',')
          raw_microsecond.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 3:
        if (*msg != '\n' && *msg != ',')
          raw_checksum.concat(*msg);
        break;
    }
    msg++;
  }

  if ((uint8_t)generateChecksum(temp, checksum_size) == raw_checksum.toInt()){
    long recieved_time = micros();
#ifdef DEBUG
    Serial.println("Valid time message");
    Serial.println(raw_second);
    Serial.println(raw_microsecond);
#endif
    int_second = raw_second.toInt();
    int_microsecond = raw_microsecond.toInt();

    int_microsecond = int_microsecond;
    if (int_microsecond > 1000000){
      int_microsecond %= 1000000;
      int_second += int_microsecond / 1000000;
    }
    sensor_clock.setTime(int_second, int_microsecond);
  }
#ifdef DEBUG
  else{
    Serial.println("Invalid time message");
    for (int i = 0; i < checksum_size; i++)
      Serial.print(temp[i]);

    Serial.println();
    Serial.println(raw_checksum);
    Serial.println(generateChecksum(temp, checksum_size));
    Serial.println("Bye");
  }
#endif
}

//Handle incoming commands
/*
ISR(USART_RX_vect){
  cli();
  char temp = UDR0;
  if (temp == '\n'){
    input_string[input_string_index] = temp;
    sei();
    if (input_string[0] == 'r')
      extractData(&input_string[0], input_string_index);
    else if (input_string[0] == 't')
      recieveTime(&input_string[0], input_string_index);
    input_string_index = 0;
    input_string_complete = true;
  }
  else{
    input_string_complete = false;
    input_string[input_string_index] = temp;
    input_string_index++;
  }
  sei();
}*/
