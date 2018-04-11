/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
*/  
#include "Arduino.h"
#include "main_mcu.hpp"
#include <TaskScheduler.h>

Scheduler ts;
LearnerCar car;
Clock sensor_clock;
volatile UltrasonicArray ultrasonic_sensors;
Encoder encoder;
Adafruit_BNO055 IMU = Adafruit_BNO055();

Task tIMU(FREQ_TO_MS(IMU_UPDATE_FREQ), TASK_FOREVER, &SendIMUDataCb, &ts, false);
Task tBatteryLevel(FREQ_TO_MS(BATTERY_LEVEL_UPDATE_FREQ), TASK_FOREVER, &SendBatteryLevelCb, &ts, false);
Task tUltrasonicData(FREQ_TO_MS(ULTRASONIC_FREQ), TASK_FOREVER,(TaskCallback) &UltrasonicDataCb, &ts, false);

void setup()   {
  //lidar negotiation pins
  pinMode(A0, INPUT);//BROWN
  pinMode(A1, OUTPUT);//BLUE
  Serial.begin(115200);
  //sei();
  //Wire.begin();

  car.Setup();
  encoder.Setup();
  ultrasonic_sensors.Setup();
  Serial.println("Hello World1");
  if(!IMU.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  //enable the tasks to be run
  //tIMU.enable();
  tBatteryLevel.enable();
  tUltrasonicData.enable();
}

void loop() {
  //sensor_clock.requestTime();
  ts.execute();
}
