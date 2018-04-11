/*
  inspiration below.
  http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/

#ifndef ULTRASONIC_ARRAY_H
#define ULTRASONIC_ARRAY_H

#include "Arduino.h"
#include <NewPing.h>
#include <Wire.h>

#define MCU1_I2C            1
#define MCU2_I2C            2
#define SONAR_CONNECTED     3
#define SONAR_NUM           7
#define FILTER_ARRAY_SIZE   3
#define MAX_DISTANCE        250 //in cm
#define ULTRASONIC_DELAY    18
#define SENSOR_WAVE_DELAY   20

class UltrasonicArray{
    private:
        NewPing* sonar_[SONAR_CONNECTED];
        uint8_t raw_distance_[SONAR_NUM];
        uint8_t unfiltered_ultrasonic_data_[SONAR_NUM][FILTER_ARRAY_SIZE];
        uint8_t last_measurement_[SONAR_NUM];
        uint8_t filtered_distance_[SONAR_NUM];
        void FilterUltrasonicData(uint8_t first_index, uint8_t last_index);
    public:
        UltrasonicArray();
        bool Setup();
        uint8_t GetUltrasonicDelay();
        void ReadFirstWave();
        void ReadSecondWave();
        void ReadThirdWave();
        void UltrasonicDataCb();
};
#endif