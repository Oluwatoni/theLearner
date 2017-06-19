int readEncoderData(){
  float dt;
  int counts;
  Wire.beginTransmission(ENCODER_ADDRESS); // transmit to device #44 (0x2c)
  Wire.write('r');  
  Wire.endTransmission();
  Wire.requestFrom(ENCODER_ADDRESS,1);
  uint8_t c[3] = {0,0,0};
  int i = 0;
  
  while(Wire.available()){
    c[i] = Wire.read();    // receive a byte as character
    i++;
  }
  //dt = ((uint16_t) c[2] + (c[1] << 8))/10000000.0;//in s
  if (Car.getDirection() == 1)
    counts = c[0];
  else if(Car.getDirection() == -1)
    counts = c[0] * -1;
  else
    counts = 0;
  
  return counts;
}

void speed_control(int encoder_ticks){
  static long prev_time = 0;
  float dt = 0.0;
  static double raw_vehicle_speed = 0.0, filtered_vehicle_speed = 0.0, filter_factor = 0.4, old_speed = 0;
  String msg;

  dt = (millis() - prev_time)/1000.0;
  raw_vehicle_speed = ((encoder_ticks / TICKS_PER_REVOLUTION) * WHEEL_CIRCUM) / dt;
  filtered_vehicle_speed = (1-filter_factor) * raw_vehicle_speed + filtered_vehicle_speed * filter_factor;
  //rename the PID variables
  Input = filtered_vehicle_speed;
  myPID.Compute();
  prev_time = millis();

  Car.Instruct(steering, Output);

  msg ="z,";
  msg.concat(Setpoint);
  msg.concat(",");
  msg.concat(filtered_vehicle_speed);
  msg.concat(",");
  sensor_clock.updateTime();
  msg = sensor_clock.appendTime(msg);
  msg = appendChecksum(msg);
#ifdef PRINT_DATA
  Serial.println(msg);
#endif
}

