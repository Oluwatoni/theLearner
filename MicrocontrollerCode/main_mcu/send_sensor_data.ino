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
  String msg;
  int count = 0;

  sensor_clock.updateTime();
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  
  //send the data
  while ((digitalRead(A0)) && (count++ < TO_MILLISECONDS(1))){}
    if (count > TO_MILLISECONDS(1)){
    return;
  }

  PORTC |= (1 << PINC1);
#ifdef PRINT_DATA
   PrintCharArray("01234567", 8, 1);
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
  msg.concat(euler.z());
  //msg.concat(car.GetBatteryLevel());
  sendData(msg);
}

void SendBatteryLevel(){
  
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

void sendUltrasonicData1(){
  String msg;

  ultrasonicRead1();
  sensor_clock.updateTime();
  msg ="u,";
  msg.concat(filtered_distance[0]);
  msg.concat(",");
  msg.concat(filtered_distance[1]);
  msg.concat(",");
  msg.concat(filtered_distance[2]);
  msg.concat(",");
  //sendData(msg);
}

void sendUltrasonicData2(){
  String msg;

  ultrasonicRead2();
  sensor_clock.updateTime();
  msg ="v,";
  msg.concat(filtered_distance[3]);
  msg.concat(",");
  msg.concat(filtered_distance[4]);
  msg.concat(",");
  //sendData(msg);
}

void sendUltrasonicData3(){
  String msg;
  long start_timer;

  start_timer = micros();
  ultrasonicRead3();
  sensor_clock.updateTime();
  msg ="w,";
  msg.concat(filtered_distance[5]);
  msg.concat(",");
  msg.concat(filtered_distance[6]);
  msg.concat(",");
  //sendData(msg);
}*/
