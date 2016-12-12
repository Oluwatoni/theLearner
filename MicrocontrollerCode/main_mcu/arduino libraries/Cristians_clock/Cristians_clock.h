#include "Arduino.h"
#ifndef CRISTIANS_CLOCK_H
#define CRISTIANS_CLOCK_H

#include <inttypes.h>

class Clock
{
  private:
    uint32_t second;
    uint32_t microsecond;
    uint64_t last_update;
  public:
    Clock();
    long request_sent;
    String appendTime(String);
    void updateTime();
    void setTime(uint32_t, uint32_t);
    void requestTime();
};

extern String appendChecksum(String);
extern char generateChecksum(char*, char);
#endif
