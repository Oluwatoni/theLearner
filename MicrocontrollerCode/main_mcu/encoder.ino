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

