#ifndef ENCODER_H
#define ENCODER_H

#define ENCODER_ADDRESS 0x07
#define WHEEL_CIRCUM 0.187
#define TICKS_PER_REVOLUTION 40.0

#include "Arduino.h"
#include <Wire.h>

class Encoder{
    private:
        int encoder_ticks_;
        long prev_time_;
        float current_speed_;
    public:
        Encoder();
        bool Setup();
        uint16_t ReadEncoderData();
        float GetCurrentSpeed();
};
#endif