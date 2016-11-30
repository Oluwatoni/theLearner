void ultrasonicRead1(){
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
  raw_distance[0] = sonar[2].ping_cm();//ultrasonic sensor 6
  
  now  = millis();
  if ((now - temp ) < ULTRASONIC_DELAY && (int)(now - temp) > 0){
    delay((temp + ULTRASONIC_DELAY) - now); //make sure 15ms has elapsed before retrieving range data //TODO make define
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[1] = Wire.read();
  
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0);
  raw_distance[2] = Wire.read();
  //end of first wave of sensor data

}

void ultrasonicRead2(){
  //beginning of second wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0);
  Wire.read();

  temp = millis();
  raw_distance[3] = sonar[0].ping_cm();// ultrasonic sensor 5
  now  = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
  {
    delay((temp + ULTRASONIC_DELAY) - now);
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[4] = Wire.read();
  //end of second wave of sensor data

}

void ultrasonicRead3(){
  //beginning of third wave
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  raw_distance[5] = sonar[1].ping_cm();//ultrasonic sensor 7
  now = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + ULTRASONIC_DELAY) - now);
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  raw_distance[6] = Wire.read();
  //end of third wave of sensor data
}

inline void swap(uint8_t* one, uint8_t* two) {
  uint8_t temp = *one;
  *one = *two;
  *two = temp;
}

uint8_t getMedian(unsigned char myArray[FILTER_ARRAY_SIZE]) {
  bool same = true;

  for (int i = 0; i < FILTER_ARRAY_SIZE; i++) {
    if (myArray[0] != myArray[i])
      same = false;
  }

  if (!same) {
    sort_array(myArray, FILTER_ARRAY_SIZE);
    return myArray[FILTER_ARRAY_SIZE / 2];
  }
  else
    return myArray[0];
}

void sort_array(uint8_t * first_index, uint8_t size_of_array) {
  int j;
  for (int i = 1; i < size_of_array; i++) {
    j = i;
    while ((j > 0) && (first_index[j - 1] > first_index[j])) {
      swap((first_index + j - 1), (first_index + j));
      j = j - 1;
    }
  }
}

void filterUltrasonicData(uint8_t first_index, uint8_t last_index) {  // Sensor ping cycle complete, do something with the results.
  for (int i = first_index; i <= last_index; i++) {
    unfiltered_ultrasonic_data[i][last_measurement[i]] = raw_distance[i];
    last_measurement[i]++;
    if (last_measurement[i] == FILTER_ARRAY_SIZE)
      last_measurement[i] = 0;
    filtered_distance[i] = getMedian(unfiltered_ultrasonic_data[i]);
  }
}
//ensures the echoes die down before pinging the sensors
void ultrasonicDelay(unsigned long now, unsigned long temp){
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + SENSOR_WAVE_DELAY) - now);
}

