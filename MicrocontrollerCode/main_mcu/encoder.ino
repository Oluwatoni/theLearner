int readEncoderData(){
  float dt;
  int speed;
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(ENCODER_ADDRESS); // transmit to device #44 (0x2c)
  Wire.write('r');  
  Wire.endTransmission();
  Wire.requestFrom(ENCODER_ADDRESS,4);
  uint8_t c[4] = {0,0,0,0};
  int i = 0;
  while(Wire.available()){ 
    c[i] = Wire.read();    // receive a byte as character
    i++;
  }
  //dt = ((uint16_t) c[3] + (c[2] << 8))/10000000.0;//in s
  speed = (c[0]);
  return speed;
}

