#include "Wire.h"
#include "Servo.h"
#include "IMU.h"
#include "Learner_car.h"

Learner_car Car;
IMU Imu(50);
void setup() {
  Serial.begin(115200);
  Imu.Setup();
  Car.Setup();
}

void loop() {
  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      long start_time= millis();
      Imu.UpdateIMU();
      int test_throttle = (Serial.readStringUntil(',')).toInt();
      Serial.println(millis()- start_time);
      Serial.print("\t");
      Serial.print((Imu.GetAccX()/255) * 9.81);
      while((millis() - start_time) < 5000)
      {
        delay(50);
        Imu.UpdateIMU();
        Car.Instruct(0,test_throttle);
        Serial.println(millis()- start_time);
        Serial.print("\t");
        Serial.print((Imu.GetAccX()/255) * 9.81);  
      }
      while((millis() - start_time) < 7000 )
      {
        delay(50);
        Imu.UpdateIMU();
        Car.Instruct(0,test_throttle+1);
        Serial.println(millis()- start_time);
        Serial.print("\t");
        Serial.print((Imu.GetAccX()/255) * 9.81);
      }
      Car.StopMove();
    }
  }
  
}
