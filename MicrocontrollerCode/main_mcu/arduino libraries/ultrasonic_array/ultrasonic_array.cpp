#include "ultrasonic_array.h"

UltrasonicArray::UltrasonicArray(){
  sonar_[0] = new NewPing(7, 7, MAX_DISTANCE); //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  sonar_[1] = new  NewPing(A2, A2, MAX_DISTANCE);//ultra 7
  sonar_[2] = new NewPing(2, 2, MAX_DISTANCE);//ultra 6

  for (int i = 0; i < SONAR_NUM; i++) {
    filtered_distance_[i] = 0;
    raw_distance_[i] = 0;
    for (int j = 0; j < FILTER_ARRAY_SIZE; j++)
      unfiltered_ultrasonic_data_[i][j] = 0;
  }
}

bool UltrasonicArray::Setup(){
  Wire.begin();
}

inline void Swap(uint8_t* one, uint8_t* two) {
  uint8_t temp = *one;
  *one = *two;
  *two = temp;
}

void SortArray(uint8_t * first_index, uint8_t size_of_array) {
  int j;
  for (int i = 1; i < size_of_array; i++) {
    j = i;
    while ((j > 0) && (first_index[j - 1] > first_index[j])) {
      Swap((first_index + j - 1), (first_index + j));
      j = j - 1;
    }
  }
}

uint8_t GetMedian(unsigned char myArray[FILTER_ARRAY_SIZE]) {
  bool same = true;

  for (int i = 0; i < FILTER_ARRAY_SIZE; i++) {
    if (myArray[0] != myArray[i])
      same = false;
  }

  if (!same) {
    SortArray(myArray, FILTER_ARRAY_SIZE);
    return myArray[FILTER_ARRAY_SIZE / 2];
  }
  else
    return myArray[0];
}

void UltrasonicArray::FilterUltrasonicData(uint8_t first_index, uint8_t last_index) {  // Sensor ping cycle complete, do something with the results.
  for (int i = first_index; i <= last_index; i++) {
    unfiltered_ultrasonic_data_[i][last_measurement_[i]] = raw_distance_[i];
    last_measurement_[i]++;
    if (last_measurement_[i] == FILTER_ARRAY_SIZE)
      last_measurement_[i] = 0;
    filtered_distance_[i] = GetMedian(unfiltered_ultrasonic_data_[i]);
  }
}

void UltrasonicArray::ReadFirstWave(){
  long temp, now;
  Wire.beginTransmission(MCU1_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  Wire.beginTransmission(MCU2_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  //beginning of first wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0);
  Wire.read();
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0);
  Wire.read();

  temp = millis();
  raw_distance_[0] = sonar_[2]->ping_cm();//ultrasonic sensor 6
  
  now  = millis();
  if ((now - temp ) < ULTRASONIC_DELAY && (int)(now - temp) > 0){
    delay((temp + ULTRASONIC_DELAY) - now); 
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0);
  raw_distance_[1] = Wire.read();
  
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0);
  raw_distance_[2] = Wire.read();
  //end of first wave of sensor data
  FilterUltrasonicData(0, 2);
  //dist_1 = filtered_distance_[0];
  //dist_2 = filtered_distance_[1];
  //dist_3 = filtered_distance_[2];
  Serial.print("u1,");
  Serial.print(filtered_distance_[0]);
  Serial.print(',');
  Serial.print(filtered_distance_[1]);
  Serial.print(',');
  Serial.println(filtered_distance_[2]);
}

void UltrasonicArray::ReadSecondWave(){
  long temp, now;
  //beginning of second wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0);
  Wire.read();

  temp = millis();
  raw_distance_[3] = sonar_[0]->ping_cm();// ultrasonic sensor 5
  now  = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0){
    delay((temp + ULTRASONIC_DELAY) - now);
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0);
  raw_distance_[4] = Wire.read();
  //end of second wave of sensor data
  FilterUltrasonicData(3, 4);
  //dist_1 = filtered_distance_[3];
  //dist_2 = filtered_distance_[4];
  Serial.print("u2,");
  Serial.print(filtered_distance_[3]);
  Serial.print(',');
  Serial.println(filtered_distance_[4]);
}

void UltrasonicArray::ReadThirdWave(){
  long temp, now;
  //beginning of third wave
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  raw_distance_[5] = sonar_[1]->ping_cm();//ultrasonic sensor 7
  now = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + ULTRASONIC_DELAY) - now);
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0);
  raw_distance_[6] = Wire.read();
  //end of third wave of sensor data
  FilterUltrasonicData(5, 6);
  Serial.print("u3,");
  Serial.print(filtered_distance_[5]);
  Serial.print(',');
  Serial.println(filtered_distance_[6]);
  //dist_1 = filtered_distance_[5];
  //dist_2 = filtered_distance_[6];
}