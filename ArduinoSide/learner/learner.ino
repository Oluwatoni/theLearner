/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
  inspiration below.
  http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/
#include <Wire.h>
#include <IMU.h>
#include <NewPing.h>
#include <Servo.h>
#include <Learner_car.h>
//#define DEBUG
//defines for the ultrasonic sensors
#define MCU1_I2C 1
#define MCU2_I2C 2
#define SONAR_NUM 7
#define MAX_DISTANCE 250 //in cm
#define ULTRASONIC_DELAY 14
#define SENSOR_WAVE_DELAY 20
#define PRECISE 3

byte raw_distance[SONAR_NUM] = {};                   // Where the range data is stored
byte filtering_distance [SONAR_NUM][PRECISE] = {};
byte filtered_distance [SONAR_NUM] = {};
byte start_filter = 0;

IMU Imu(50);
Learner_car Car;
NewPing sonar[SONAR_NUM - 4] =       // Sensor object array.
{
  NewPing(7, 8, MAX_DISTANCE), //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE),//ultra 7
  NewPing(4, 2, MAX_DISTANCE)//ultra 6
};

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  Imu.Setup();
  Car.Setup();
  Serial.begin(115200);
}

unsigned long temp = 0, now  =0;
float freq;
//checksum to help validate msg

uint16_t generateChecksum(char data[], byte sizeOfData)
{
  uint16_t sum = 0;
  while (sizeOfData)
  {
    sum += (uint8_t)data[--sizeOfData];
  }
  sum += 44;
  return (sum % 255);
}

void loop()
{
  unsigned long startTime  = millis();
  ultrasonicRead();
  temp = millis();
  filterUltrasonicData();
  Imu.UpdateIMU();
  String msg;
  msg.concat(filtered_distance[0]);
  msg.concat(",");
  msg.concat(filtered_distance[1]);
  msg.concat(",");
  msg.concat(filtered_distance[2]);
  msg.concat(",");
  msg.concat(filtered_distance[3]);
  msg.concat(",");
  msg.concat(filtered_distance[4]);
  msg.concat(",");
  msg.concat(filtered_distance[5]);
  msg.concat(",");
  msg.concat(filtered_distance[6]);
  msg.concat(",");
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
  msg.concat(freq);
  msg.concat(",");
  msg.concat(millis());
  msg.concat(",");
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
  now = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + SENSOR_WAVE_DELAY) - now);
  freq = 1.0 / ((millis() - startTime) / 1000.0);
}

//handles RC msgs
void serialEvent() {
  int steering, throttle, stopRequested,checksum;
  String to_be_checked = "";
  inputString = "";
    delay(10);
    inputString = Serial.readStringUntil(',');
    if (inputString.startsWith("r"))
    {
      to_be_checked.concat("r,");
      steering = (inputString.substring(1)).toInt();
      to_be_checked.concat(steering);
      to_be_checked.concat(",");
      inputString = Serial.readStringUntil(',');
      throttle = inputString.toInt();
      inputString = Serial.readStringUntil(',');
      to_be_checked.concat(throttle);
      to_be_checked.concat(",");
      stopRequested = inputString.toInt();
      to_be_checked.concat(stopRequested);
      to_be_checked.concat(",");
      inputString = Serial.readStringUntil('\n');
//      Serial.println(to_be_checked);
      checksum = inputString.toInt();
#ifdef DEBUG
      digitalWrite(13, HIGH);
      String debug_msg = "";
      debug_msg.concat(steering);
      debug_msg.concat(",");
      debug_msg.concat(throttle);
      Serial.println(debug_msg);
#endif
    }
    char buffer[to_be_checked.length()];
    to_be_checked.toCharArray(buffer, to_be_checked.length());
    if (checksum == generateChecksum(buffer, to_be_checked.length()))
      Car.Instruct(steering,throttle);
    else
    {
      Serial.print(generateChecksum(buffer, to_be_checked.length()));
      Serial.print(", ");
      Serial.println(checksum);
    }
}


