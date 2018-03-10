#include "Arduino.h"
#ifndef LEARNER_CAR_H
#define LEARNER_CAR_H

#include <Servo.h>
#include <PID_v1.h>

#define STEERING_NEUTRAL 75
#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define STEERING_SERVO_PIN 3
#define TEST_LED_PIN 13
#define BATTERY_MONITOR_PIN A7

class LearnerCar{
private:
	int8_t steering_;
	int8_t throttle_;
	Servo steering_servo_;
	char direction_;
	int battery_level_;
	PID* linear_velocity_pid_;
	double velocity_pid_input_;
	double velocity_pid_output_;
	double velocity_pid_setpoint_;
	double kp_;
	double ki_;
	double kd_;
	enum driving_state {
		REVERSING = -1,
		STOP = 0,
		FORWARD = 1
	};

	void Forward(int8_t speedy);
	void Reverse(int8_t speedy);
	void StopMove();
	void SoftStop();
	void Instruct(int8_t dir, int8_t power);//direction (-100 to 100) power 0 to 100
public:
	LearnerCar();
	~LearnerCar();
	bool Setup();
	int GetBatteryLevel();
	void SetVelocities(int8_t steering, int8_t throttle);//TODO change to velocities!
  int8_t GetDirection();
  void UpdateWheels(float current_speed);
};

#endif