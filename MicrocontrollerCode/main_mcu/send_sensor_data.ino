/*
float f = 0;
float mean = 0;
long start = millis();
f = 0.98;
mean = (mean * f) + (1.0-f) * (1000.0/float(millis()-start));
start = millis();
Serial.println(mean,4);
*/
#define F_CPU 16000000
#define TO_MILLISECONDS(x) (x)*(F_CPU / 1000)
String msg;

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

void sendImuData(){
  Imu.UpdateIMU();
  sensor_clock.updateTime();
  msg ="i,";
  msg.concat(Imu.GetYaw());
  msg.concat(Imu.GetGyroZ());
  msg.concat(Car.GetBatteryLevel());
  sendData(msg);
}

void sendEncData(){
  long start_timer, cnt;
  start_timer = micros();
  msg ="e";
  cnt = readEncoderData();
  msg.concat(cnt);//X_axis
  sendData(msg);
  speed_control(cnt);
}

void sendUltrasonicData1(){
  ultrasonicRead1();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(0, 2);
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
  ultrasonicRead2();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(3, 4);
  msg ="v,";
  msg.concat(filtered_distance[3]);
  msg.concat(",");
  msg.concat(filtered_distance[4]);
  msg.concat(",");
  //sendData(msg);
}

void sendUltrasonicData3(){
  long start_timer;
  start_timer = micros();
  ultrasonicRead3();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(5, 6);
  msg ="w,";
  msg.concat(filtered_distance[5]);
  msg.concat(",");
  msg.concat(filtered_distance[6]);
  msg.concat(",");
  //sendData(msg);
}
