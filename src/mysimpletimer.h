/*
  
*/
#ifndef mysimpletimer_h
#define mysimpletimer_h

#include <Arduino.h>

class MySimpleTimer
{
public:
    MySimpleTimer();
    void setIntervalMs(uint32_t intervalMs);
    void resetTimingNow (uint32_t nowMs);
    int32_t getStatus (uint32_t nowMs, bool resetOnTrigger = true);
private:
    uint32_t _lasttime = 0;
    uint32_t _interval = 0;
};

#endif
