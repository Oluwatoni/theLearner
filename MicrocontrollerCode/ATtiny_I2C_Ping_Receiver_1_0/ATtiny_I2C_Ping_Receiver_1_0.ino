/*

Oluwatoni Ogunmade 

inspiration below.
http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/

#include <Wire.h>
#include <NewPing.h>

#define MCU1_I2C 1
#define MCU2_I2C 2
#define SONAR_NUM 3
#define MAX_DISTANCE 250 //in cm

byte Distance[7]={};                              // Where the Distance is stored (8 bit unsigned)

NewPing sonar[SONAR_NUM] =       // Sensor object array.
{
  NewPing(7, 8, MAX_DISTANCE), //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE),//ultra 7 
  NewPing(4, 2, MAX_DISTANCE)//ultra 6
};

void setup()
{
  Wire.begin();
  delay(200);
  Serial.begin(57600);
  Serial.println("Setup");
}

void loop()
{
  unsigned long startTime  = millis();
  Wire.beginTransmission(MCU1_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();
  
  Wire.beginTransmission(MCU2_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();
  
  //beginning of first wave
  //MCU1
//  Serial.println("state 0");
  Wire.requestFrom(MCU1_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();
  
//  Serial.println("state 2");
  unsigned long temp = millis();
  Distance[5] = sonar[2].ping_cm();//ultrasonic sensor 6
  if (millis()- temp > 0)
    delay((temp+18)-millis()); //make sure 15ms has elapsed before retrieving range data //TODO make define
  
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
//  Serial.println("state 3");
  while (Wire.available() == 0)  ;
  Distance[0] = Wire.read();
  
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
//  Serial.println("state 3");
  while (Wire.available() == 0)  ;
  Distance[3] = Wire.read();
  //end of first wave of sensor data
  
  delay(25);
  //beginning of second wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();   
  
//  Serial.println("state 5");
  temp = millis();
  Distance[4] = sonar[0].ping_cm();// ultrasonic sensor 5
  if (millis()- temp > 0)
    delay((temp+18)-millis());
  
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[1] = Wire.read();   
  //end of second wave of sensor data
  
  delay(25);
  //beginning of third wave
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();   
  
//  Serial.println("state 5");
  temp = millis();
  Distance[6] = sonar[1].ping_cm();//ultrasonic sensor 7
  if (millis()- temp > 0)
    delay((temp+18)-millis());
  
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[2] = Wire.read();   
  //end of third wave of sensor data
//  Serial.println("end");
  float freq = 1.0/((millis() - startTime)/1000.0);
  String msg = "1: ";
  msg.concat(Distance[0]);
  msg.concat(", 2: ");
  msg.concat(Distance[1]);
  msg.concat(", 3: ");
  msg.concat(Distance[2]);
  msg.concat(", 4: ");
  msg.concat(Distance[3]);
  msg.concat(", 5: ");
  msg.concat(Distance[4]);
  msg.concat(", 6: ");
  msg.concat(Distance[5]);
  msg.concat(", 7: ");
  msg.concat(Distance[6]);
  msg.concat(", frequency: ");
  msg.concat(freq);
  msg.concat(", ");
  msg.concat(millis());
  Serial.println(msg);
  delay(25);
}
