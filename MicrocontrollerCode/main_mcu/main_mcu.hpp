/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
*/  

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <avr/interrupt.h>
#include <Cristians_clock.h>
#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskSchedulerDeclarations.h>
#include <learner_car.h>
#include <encoder.h>
#include <Wire.h>
#include <ultrasonic_array.h>

#define UART_RX_HARDWARE_ENABLE
#define PRINT_DATA
#define FREQ_TO_MS(X) ((1/X) * 1000)
#define TO_MILLISECONDS(x) (x)*(F_CPU / 1000)

const int IMU_UPDATE_FREQ = 50;
const int BATTERY_LEVEL_UPDATE_FREQ = 1;
const int ULTRASONIC_FREQ = 1;

extern Scheduler ts;
/* tasks to be run
- ultrasonic array coupled with a local map update
- IMU
- car controller (PID and regular controls)
- motor easing
- time request
*/

extern LearnerCar car;
extern Clock sensor_clock;
extern volatile UltrasonicArray ultrasonic_sensors;
extern Encoder encoder;
extern Adafruit_BNO055 IMU;

extern Task tIMU;
extern Task tBatteryLevel;
extern Task second_wave_timeout;

//callbacks
void SendIMUDataCb();
void SendBatteryLevelCb();
void UltrasonicDataCb();
void UpdateCarCb();
void TimeRequestCb();
void UltrasonicDataCb();
