#include "encoder.h"

Encoder::Encoder(){
    encoder_ticks_ = 0;
    prev_time_ = millis();
}

bool Encoder::Setup(){
    Wire.begin();
    return true;
}

uint16_t Encoder::ReadEncoderData(){
    static double raw_vehicle_speed = 0.0, filtered_vehicle_speed = 0.0, filter_factor = 0.4, old_speed = 0;
    float dt = 0;
    int counts = 0;

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
    counts = 0;

    dt = (millis() - prev_time_)/1000.0;
    raw_vehicle_speed = ((counts / TICKS_PER_REVOLUTION) * WHEEL_CIRCUM) / dt;
    current_speed_ = (1-filter_factor) * raw_vehicle_speed + filtered_vehicle_speed * filter_factor;
    prev_time_ = millis();

    return counts;
}

float Encoder::GetCurrentSpeed(){
  return current_speed_;
}