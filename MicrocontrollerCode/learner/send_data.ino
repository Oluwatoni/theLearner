String appendChecksum(String msg)
{
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  return msg;
}

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

Clock:: Clock()
{
  second = 0;
  microsecond = 0;
  last_update = micros();
}

//request time to set up the internal clock
void Clock:: requestTime()
{
  String msg;
  msg.concat("t,now,");
  msg = appendChecksum(msg);
  Serial.println(msg);
  request_sent = micros();
}

void Clock:: setTime( uint32_t init_second, uint32_t init_microsecond)
{
  second = init_second;
  microsecond = init_microsecond;
  last_update = micros();
#ifdef DEBUG
  Serial.println(second);
  Serial.println(microsecond);
#endif
}

void Clock:: updateTime()
{
  long current_update = micros() - last_update;
  second += current_update /1000000;
  microsecond += current_update % 1000000;
  if (microsecond > 1000000)
  {
    microsecond %= 1000000;
    second += microsecond/1000000;
  } 
  last_update = micros();
}

String Clock:: appendTime(String msg)
{
  msg.concat(sensor_clock.second);
  msg.concat(",");
  msg.concat(sensor_clock.microsecond);
  msg.concat(",");
  return msg;
}
