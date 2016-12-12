#include "Cristians_clock.h"
Clock:: Clock(){
  second = 0;
  microsecond = 0;
  last_update = micros();
}

//request time to set up the internal clock
void Clock:: requestTime(){
  String msg;
  msg.concat("t,now,");
  msg = appendChecksum(msg);
  Serial.println(msg);
  request_sent = micros();
}

void Clock:: setTime( uint32_t init_second, uint32_t init_microsecond){
  second = init_second;
  microsecond = init_microsecond;
  last_update = micros();
#ifdef DEBUG
  Serial.println(second);
  Serial.println(microsecond);
#endif
}

void Clock:: updateTime(){
  long current_update = micros() - last_update;
  microsecond += current_update;
  
  if (microsecond >= 1000000){
    microsecond %= 1000000;
    second++;
  } 
  last_update = micros();
}

String Clock:: appendTime(String msg){
  msg.concat(second);
  msg.concat(",");
  msg.concat(microsecond);
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
