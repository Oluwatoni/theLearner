
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

