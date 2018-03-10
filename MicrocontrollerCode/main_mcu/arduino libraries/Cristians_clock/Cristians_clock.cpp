#include "Cristians_clock.h"
Clock:: Clock(){
  second_ = 0;
  microsecond_ = 0;
  last_update_ = micros();
}

//request time to set up the internal clock
void Clock:: requestTime(){
  String msg;
  msg.concat("t,now,");
  msg = appendChecksum(msg);
  Serial.println(msg);
  request_sent = micros();
}

void Clock:: setTime( uint32_t init_second, uint32_t init_microsecond_){
  second_ = init_second;
  microsecond_ = init_microsecond_;
  last_update_ = micros();
#ifdef DEBUG
  Serial.println(second_);
  Serial.println(microsecond_);
#endif
}

void Clock:: updateTime(){
  long current_update = micros() - last_update_;
  microsecond_ += current_update;
  
  if (microsecond_ >= 1000000){
    microsecond_ %= 1000000;
    second_++;
  } 
  last_update_ = micros();
}

String Clock:: appendTime(String msg){
  msg.concat(second_);
  msg.concat(",");
  msg.concat(microsecond_);
  msg.concat(",");
  return msg;
}

String appendChecksum(String msg){
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat((uint8_t)generateChecksum(buffer, msg.length()));
  return msg;
}

char generateChecksum(char data[], char sizeOfData){
  uint32_t sum = 0;
  while (sizeOfData)
    sum += (uint8_t)data[--sizeOfData];
  sum += 44;
  return (sum % 255);
}
