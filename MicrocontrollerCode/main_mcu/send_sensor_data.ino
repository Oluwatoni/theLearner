/*
float f = 0;
float mean = 0;
long start = millis();
f = 0.98;
mean = (mean * f) + (1.0-f) * (1000.0/float(millis()-start));
start = millis();
Serial.println(mean,4);
*/
 String msg;
 
void sendImuData(){
  Imu.UpdateIMU();
  sensor_clock.updateTime();
  msg ="i,";
  msg.concat(Imu.GetYaw());
  msg.concat(",");
  msg.concat(Imu.GetPitch());
  msg.concat(",");
  msg.concat(Imu.GetRoll());
  msg.concat(",");
  msg.concat(Imu.GetAccX());
  msg.concat(",");
  msg.concat(Imu.GetAccY());
  msg.concat(",");
  msg.concat(Imu.GetAccZ());
  msg.concat(",");
  msg.concat(Imu.GetGyroX());
  msg.concat(",");
  msg.concat(Imu.GetGyroY());
  msg.concat(",");
  msg.concat(Imu.GetGyroZ());
  msg.concat(",");
//  msg.concat(Imu.GetMagX());
//  msg.concat(",");
//  msg.concat(Imu.GetMagY());
//  msg.concat(",");
//  msg.concat(Imu.GetMagZ());
//  msg.concat(",");
  msg.concat(Car.GetBatteryLevel());
  msg.concat(",");
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}

void sendAccData(){
  msg ="a,";
  msg.concat(analogRead(A0));//X_axis
  msg.concat(",");
  msg.concat(analogRead(A1));//Y_axis
  msg.concat(",");
  sensor_clock.updateTime();
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}

void sendEncData(){
  long start_timer;
  start_timer = micros();
  msg ="e,";
  msg.concat(readEncoderData());//X_axis
  msg.concat(",");
  sensor_clock.updateTime();
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
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
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
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
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
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
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}
