
void ultra()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  { // Loop through all the sensors.
    if (millis() >= pingTimer[i])
    { // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  //delay(30);
}

void echoCheck()   // If ping received, set the sensor distance to array.
{
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle()   // Sensor ping cycle complete, do something with the results.
{
  if (j <= (PRECISE - 1))
  {
    frontReal[j] = front;
    leftReal[j] = left;
    rightReal[j] = right;
    rearReal[j] = rear;
    j++;
  }
  else 
  {
    for (int i = 1; i < PRECISE; i++)
    {
      frontReal[i - 1] = frontReal[i];
      leftReal[i - 1] = leftReal[i];
      rightReal[i - 1] = rightReal[i];
      rearReal[i - 1] = rearReal[i];
    }
    if (front != 0)
      frontReal[(PRECISE - 1)] = front;
    if (left != 0)
      leftReal[(PRECISE - 1)] = left;
    if (right != 0)
      rightReal[(PRECISE - 1)] = right;
    if (rear != 0)
      rearReal[(PRECISE - 1)] = rear;

    front = getMedian(frontReal);
    left = getMedian(leftReal);
    right = getMedian(rightReal);
    rear = getMedian(rearReal);
  }

  rear = cm[3];
  front = cm[2];
  left = cm[1];
  right = cm[0];
}
