void updateBatteryLevel()
{
  batteryLevel = map(analogRead(BATTERY_MONITOR_PIN), 0, 1024, 0, 1024);
}

void forward(int speedy)
{
  direction = 1;
  analogWrite(REVERSE_PIN, 0);
  analogWrite(FORWARD_PIN, speedy);
}

//Add reverse functions
void reverse(int speedy)
{
  direction = -1;
  analogWrite(FORWARD_PIN, 0);
  analogWrite(REVERSE_PIN, speedy);
}

void stopMove()
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

void softStop()
{
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);
}

void instruct(int dir, int power)//direction (-100 to 100) power 0 to 100
{
  if (power > 0)
    forward(map(power, 0, 100, 90, 255));
  else if (power == 0)
    forward(0);
  else
    reverse(map(abs(power), 0, 100, 90, 255));
  steeringServo.write(map(dir, -100, 100, 85, 65));
}
