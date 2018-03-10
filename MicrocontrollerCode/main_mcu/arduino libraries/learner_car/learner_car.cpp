#include "learner_car.h"

LearnerCar::LearnerCar(){
  kp_ = 800.0;
  ki_ = 30.0;
  kd_ = 0.0;
  linear_velocity_pid_ = new PID(&velocity_pid_input_, &velocity_pid_output_, &velocity_pid_setpoint_, kp_, ki_, kd_, DIRECT);
}

LearnerCar::~LearnerCar(){
  delete linear_velocity_pid_;
}

bool LearnerCar::Setup(){
  pinMode(TEST_LED_PIN, OUTPUT);//test
  pinMode(REVERSE_PIN, OUTPUT);//reverse
  pinMode(FORWARD_PIN, OUTPUT);//forward
  pinMode(STEERING_SERVO_PIN, OUTPUT);//steering servo
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  steering_servo_.attach(STEERING_SERVO_PIN);
  steering_servo_.write(STEERING_NEUTRAL);
  linear_velocity_pid_->SetMode(MANUAL);
  linear_velocity_pid_->SetOutputLimits(-255, 255);
  return true;
}

int LearnerCar::GetBatteryLevel(){
  return analogRead(BATTERY_MONITOR_PIN);
}

void LearnerCar::Forward(int8_t speedy){
  if (speedy > 0)
    direction_ = 1;
  analogWrite(REVERSE_PIN, 0);
  analogWrite(FORWARD_PIN, speedy);
}

void LearnerCar::Reverse(int8_t speedy){
  if (speedy > 0)
    direction_ = REVERSING;
  analogWrite(FORWARD_PIN, 0);
  analogWrite(REVERSE_PIN, speedy);
}

void LearnerCar::StopMove(){
  unsigned long initTimeCount, finalTimeCount;
  if (direction_ == FORWARD){
    initTimeCount = millis();
    finalTimeCount = millis();
    while (finalTimeCount - initTimeCount <= 70){
      analogWrite(REVERSE_PIN, 0);
      analogWrite(FORWARD_PIN, 200);
      finalTimeCount = millis();
    }
    analogWrite(REVERSE_PIN, 0);
    analogWrite(FORWARD_PIN, 0);
    direction_ = REVERSING;
  }
  else  {
    analogWrite(REVERSE_PIN,  0);
    analogWrite(FORWARD_PIN, 0);
  }
}

void LearnerCar::SoftStop(){
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);
}

//direction (-100 to 100) power 0 to 100
void LearnerCar::Instruct(int8_t dir, int8_t power){
  if (abs(power) > 80)
    if (power > 0)
      Forward(power);
    else
      Reverse(abs(power));
  else
    Forward(0);
    
  steering_servo_.write(map(dir, -30, 30, 85, 65));
}

int8_t LearnerCar::GetDirection(){
  return direction_;
}

void LearnerCar::SetVelocities(int8_t steering, int8_t throttle){
  steering_ = steering;
  throttle_ = throttle_;
}

//PID velocity update
void LearnerCar::UpdateWheels(float current_speed){
  //rename the PID variables
  velocity_pid_input_ = current_speed;
  linear_velocity_pid_->Compute();
  Instruct(steering_, throttle_);
}