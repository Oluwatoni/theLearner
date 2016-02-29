#include <Wire.h>
#include <IMU.h>

IMU Imu(50.0);

void setup() {
  Serial.begin(115200);
  Imu.setup();
  
}

void loop() {
  Imu.updateIMU();
  Serial.println("Hello");
  String msg = "";
  msg.concat(Imu.GetYaw());
  msg.concat(", ");
  msg.concat(Imu.GetPitch());
  msg.concat(", ");
  msg.concat(Imu.GetRoll());
  msg.concat(", ");
  Serial.println(msg);
  delay(10);
}
