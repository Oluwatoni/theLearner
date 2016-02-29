#include "learner_car.h"
#include <Servo.h>

Learner_car::Learner_car()
{
	
}

void Learner_car::Setup()
{
  pinMode(TEST_LED_PIN, OUTPUT);//test
  pinMode(REVERSE_PIN, OUTPUT);//reverse
  pinMode(FORWARD_PIN, OUTPUT);//forward
  pinMode(STEERING_SERVO_PIN, OUTPUT);//steering servo
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  steering.attach(STEERING_SERVO_PIN);
  steering.write(STEERING_NEUTRAL);
  delay(1000);
}
int Learner_car::GetBatteryLevel()
{
  return analogRead(BATTERY_MONITOR_PIN);
}

void Learner_car::Forward(int speedy)
{
  direction = 1;
  analogWrite(REVERSE_PIN, 0);
  analogWrite(FORWARD_PIN, speedy);
}

//Add reverse functions
void Learner_car::Reverse(int speedy)
{
  direction = -1;
  analogWrite(FORWARD_PIN, 0);
  analogWrite(REVERSE_PIN, speedy);
}

void Learner_car::StopMove()
{
  unsigned long initTimeCount, finalTimeCount;
  if (direction == 1) //vehicle moving forward
  {
    initTimeCount = millis();
    finalTimeCount = millis();
    while (finalTimeCount - initTimeCount <= 70) //instead of a delay
    {
      analogWrite(REVERSE_PIN, 0);
      analogWrite(FORWARD_PIN, 200);
      finalTimeCount = millis();
    }
    analogWrite(REVERSE_PIN, 0);
    analogWrite(FORWARD_PIN, 0);
    direction = 0;
  }
  else
  {
    analogWrite(REVERSE_PIN,  0);
    analogWrite(FORWARD_PIN, 0);
  }
}

void Learner_car::SoftStop()
{
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);
}

void Learner_car::Instruct(int dir, int power)//direction (-100 to 100) power 0 to 100
{
  if (power > 0)
    Forward(map(power, 0, 100, 90, 255));
  else if (power == 0)
    Forward(0);
  else
    Reverse(map(abs(power), 0, 100, 90, 255));
  steering.write(map(dir, -100, 100, 85, 65));
}