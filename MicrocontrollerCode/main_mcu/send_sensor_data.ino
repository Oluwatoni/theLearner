void sendImuData()
{
  Imu.UpdateIMU();
  sensor_clock.updateTime();
  String msg;
  msg.concat("i,");
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
  msg.concat(Car.GetBatteryLevel());
  msg.concat(",");
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}

void sendAccData()
{
  long start_timer;
  start_timer = micros();
  String msg;
  msg.concat("a,");
  msg.concat(analogRead(A0));//X_axis
  msg.concat(",");
  msg.concat(analogRead(A1));//Y_axis
  msg.concat(",");
  sensor_clock.updateTime();
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
  start_timer = micros()-start_timer;
  Serial.println(start_timer);
}
void sendUltrasonicData1()//sends IMU data inbetween waves
{
  ultrasonicRead1();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(0, 2);
  String msg;
  msg.concat("u,");
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

void sendUltrasonicData2()//sends IMU data inbetween waves
{
  ultrasonicRead2();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(3, 4);
  String msg;
  msg.concat("v,");
  msg.concat(filtered_distance[3]);
  msg.concat(",");
  msg.concat(filtered_distance[4]);
  msg.concat(",");
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}

void sendUltrasonicData3()//sends IMU data inbetween waves
{
  ultrasonicRead3();
  sensor_clock.updateTime();
  //!filter the readings from the above call
  filterUltrasonicData(5, 6);
  String msg;
  msg.concat("w,");
  msg.concat(filtered_distance[5]);
  msg.concat(",");
  msg.concat(filtered_distance[6]);
  msg.concat(",");
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
  Serial.println(msg);
}

