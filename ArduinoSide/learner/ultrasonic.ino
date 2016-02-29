void ultrasonicRead()
{
  Wire.beginTransmission(MCU1_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  Wire.beginTransmission(MCU2_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  //beginning of first wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  raw_distance[5] = sonar[2].ping_cm();//ultrasonic sensor 6
  now  = millis();
  if ((now - temp ) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
  {
    delay((temp + ULTRASONIC_DELAY) - now); //make sure 15ms has elapsed before retrieving range data //TODO make define
  }
  
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[0] = Wire.read();

  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[3] = Wire.read();
  //end of first wave of sensor data

  delay(SENSOR_WAVE_DELAY);
  //beginning of second wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  raw_distance[4] = sonar[0].ping_cm();// ultrasonic sensor 5
  now  = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
  {
    delay((temp + ULTRASONIC_DELAY) - now);
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[1] = Wire.read();
  //end of second wave of sensor data

  delay(SENSOR_WAVE_DELAY);
  //beginning of third wave
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  raw_distance[6] = sonar[1].ping_cm();//ultrasonic sensor 7
  now = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + ULTRASONIC_DELAY) - now);
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[2] = Wire.read();
  //end of third wave of sensor data
}
byte getMedian(byte (&myArray)[PRECISE])//fixxx
{
  int highest = 0, highestIndex = 0, count = 0;
  int sort [PRECISE] = {};
  int sorted [PRECISE] = {};
  int sorted_freq [PRECISE] = {};
  bool not_same = true;

  for (int i = 0; i < PRECISE; i++)
  {
    if (myArray[0] != myArray[i])
      not_same = false;
  }

  if (not_same)
  {
    for (int i = 0; i < PRECISE; i++)
    {
      if (myArray[i] > highest)
      {
        highest = myArray[i];
        highestIndex = i;
      }
    }

    for (int k = 0; k < highest; k++)
    {
      for (int i = 0; i < PRECISE; i++)
      {
        if ((highest - k) == myArray[i])
        {
          sort[count] = myArray[i];
          for (int k = 0; k < PRECISE; k++)
          {
            if (sort[count] == myArray[k])
            {
              sorted_freq[count]++;
              count++;
            }
          }
        }
      }
    }
    count = 0;
    for (int i = 0; i < PRECISE; i++)
    {
      for (int k = 0; k < sorted_freq[i]; i++)
      {
        sorted[count] = sort[i];
        count++;
        if (count > (PRECISE - 1))
          break;
      }
    }
    return sorted[2];
  }
  else
    return myArray[1];
}

void filterUltrasonicData()   // Sensor ping cycle complete, do something with the results.
{
  if (start_filter <= (PRECISE - 1))
  {
    for (int i = 0; i < SONAR_NUM; i++)
    {
      filtering_distance[i][start_filter] = raw_distance[i];
    }

    start_filter++;
  }
  else 
  {
    for (int i = 1; i < PRECISE; i++)
    {
      for (int n = 0; n < SONAR_NUM; n++)
      {
        filtering_distance[n][i - 1] = filtering_distance[n][i];
      }
    }
      
    for (int n = 0; n < SONAR_NUM; n++)
    {
      filtering_distance[n][PRECISE - 1] = raw_distance[n];
    }
    for (int n = 0; n < SONAR_NUM; n++)
    {
      filtered_distance[n] = getMedian(filtering_distance[n]);
    }
  }
}
