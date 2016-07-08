void sendImuData()
{
  Imu.UpdateIMU();
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
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
}

void sendUltrasonicData1()//sends IMU data inbetween waves
{
  ultrasonicRead1();
  //!filter the readings from the above call
  filterUltrasonicData(0,2);
  String msg;
  msg.concat("u,");
  msg.concat(filtered_distance[0]);
  msg.concat(",");
  msg.concat(filtered_distance[1]);
  msg.concat(",");
  msg.concat(filtered_distance[2]);
  msg.concat(",");
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
}

void sendUltrasonicData2()//sends IMU data inbetween waves
{
  ultrasonicRead2();
  //!filter the readings from the above call
  filterUltrasonicData(3,4);
  String msg;
  msg.concat("v,");
  msg.concat(filtered_distance[3]);
  msg.concat(",");
  msg.concat(filtered_distance[4]);
  msg.concat(",");
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
}

void sendUltrasonicData3()//sends IMU data inbetween waves
{
  ultrasonicRead3();
  //!filter the readings from the above call
  filterUltrasonicData(5,6);
  String msg;
  msg.concat("w,");
  msg.concat(filtered_distance[5]);
  msg.concat(",");
  msg.concat(filtered_distance[6]);
  msg.concat(",");
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
}

void sendGPSData()
{
  /*a-latitude
    b-longitude
    c-time
    d-speed
    e-course
  */
  String GPS_msg = "g,";
  byte len[5] = {12,13,9,5,5};
  char req = 'a';
  for (int i = 0; i< 5;i++)
  {
    Wire.beginTransmission(0x10);
    Wire.write((char)req+i);
    Wire.endTransmission();
    Wire.requestFrom(0x10,len[i]);
    
    for(int j = 0; j < len[i]; j++)
    {
      while (!Wire.available());
      char temp = Wire.read();
      //if (temp != ' ')
      GPS_msg.concat(temp);
    }
    GPS_msg.concat(',');  
    // echo out whatever we get back
  }
  Serial.println(GPS_msg);
}

