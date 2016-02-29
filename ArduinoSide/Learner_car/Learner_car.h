#include "Arduino.h"
#ifndef LEARNER_CAR_H
#define LEARNER_CAR_H
#include <Servo.h>

#define STEERING_NEUTRAL 75
#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define STEERING_SERVO_PIN 3
#define TEST_LED_PIN 13
#define BATTERY_MONITOR_PIN A7

class Learner_car
{
private:
	Servo steering;
	bool direction;
	int battery_level;
public:
	Learner_car();
	void Setup();
	int GetBatteryLevel();
	void Forward(int speedy);
	void Reverse(int speedy);
	void StopMove();
	void SoftStop();
	void Instruct(int dir, int power);//direction (-100 to 100) power 0 to 100
};

#endif