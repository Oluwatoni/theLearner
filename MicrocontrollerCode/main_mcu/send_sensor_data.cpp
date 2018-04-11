#include "Arduino.h"
#include "main_mcu.hpp"

void UART_Transmit( unsigned char data ){
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

void PrintCharArray(char array[], uint8_t size, uint8_t newline){
  for (uint8_t i = 0;i < size; i++)
    UART_Transmit(*(array++));
  if (newline){
    //UART_Transmit('\r');
    UART_Transmit('\n');
  }
}

void sendData(String data){
  String msg = data;
  int count = 0;

  sensor_clock.updateTime();
  //msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  
  //send the data
  while ((digitalRead(A0)) && (count++ < TO_MILLISECONDS(1))){}
    if (count > TO_MILLISECONDS(1)){
    return;
  }

  PORTC |= (1 << PINC1);
#ifdef PRINT_DATA
   PrintCharArray(msg.c_str(), msg.length(), 1);
   Serial.flush();
#endif
  delayMicroseconds(200);
  PORTC &= !(1 << PINC1);
}

void SendIMUDataCb(){
  String msg;
  
  imu::Vector<3> euler;
  euler = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  sensor_clock.updateTime();
  msg ="i,";
  msg.concat(euler.x());
  msg.concat(',');
  msg.concat(euler.z());
  //sendData(msg);
}

void SendBatteryLevelCb(){
  String msg;
  msg ="b,";
  msg.concat(car.GetBatteryLevel());
  sendData(msg);
}

void UltrasonicDataCb(){
  static int callback_wave_ = 0;
  switch(callback_wave_){
    case 0:
      ultrasonic_sensors.ReadFirstWave();
      break;
    case 1:
      ultrasonic_sensors.ReadSecondWave();
      break;
    case 2:
      ultrasonic_sensors.ReadThirdWave();
      break;
  }
  callback_wave_++;
  if (callback_wave_ >= 3){
    callback_wave_ = 0;
  }
}

/*
void sendEncData(){
  String msg;
  long start_timer, cnt;

  start_timer = micros();
  msg ="e";
  cnt = readEncoderData();
  msg.concat(cnt);//X_axis
  sendData(msg);
  speed_control(cnt);
}
*/
