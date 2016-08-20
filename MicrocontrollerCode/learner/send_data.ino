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
//  Serial.println(msg);
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
//  Serial.println(msg);
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
//  Serial.println(msg);
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
//  Serial.println(msg);
}

void sendGPSData()
{
  /*a-latitude
    b-longitude
    c-time
    d-speed
    e-course
    f-altitude
  */
  
  ///TODO add more character filters eg only permit (N,S,E,W and others necessary for checksum)
  String GPS_msg = "g,",GPS_msg_part;
  byte len[6] = {13,14,10,6,6,7};
  char req = 'a', temp = 0;

  for(int i = 0; i < sizeof(len);i++)
  {
    long start_time= millis();
    //repeats until i2c data of the correct size is recieved
    while(1)
    {
      if((millis()-start_time)>20)
      {
        GPS_msg_part = "ERR";
        break;
      }
      
      GPS_msg_part = "";
      Wire.beginTransmission(0x10);
      Wire.write((char)req+i);
      Wire.endTransmission();
      Wire.requestFrom(0x10,len[i]);
      
      temp = Wire.read();
      if(temp == 'E')
      {
        GPS_msg_part = "ERR";
        break;
      }
      GPS_msg_part.concat(temp);
      while (Wire.available())
      {
        temp = Wire.read();
        if(temp > 43 && temp < 122)//restricting it to numbers and letters
          GPS_msg_part.concat(temp);
      }
      GPS_msg_part = GPS_msg_part.substring(0,GPS_msg_part.length()-1);
      
      if(GPS_msg_part.length() == ((temp-'0')))
        break;
    }
    GPS_msg.concat(GPS_msg_part);
    GPS_msg.concat(',');
  }
  char buffer[GPS_msg.length()];
  GPS_msg.toCharArray(buffer, GPS_msg.length());
  GPS_msg.concat(generateChecksum(buffer, GPS_msg.length()));
  Serial.println(GPS_msg);
}

