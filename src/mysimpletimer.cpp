#include "mysimpletimer.h"

#include <Arduino.h>

MySimpleTimer::MySimpleTimer ( )
{

}

void MySimpleTimer::setIntervalMs (uint32_t intervalMs)
{
    if (intervalMs == 0){
        intervalMs = 1;
    }
    _interval = intervalMs;
}

void MySimpleTimer::resetTimingNow (uint32_t nowMs)
{
    _lasttime = nowMs;
}

int32_t MySimpleTimer::getStatus (uint32_t nowMs, bool resetOnTrigger)
{
    uint32_t actualTimeDifference = nowMs - _lasttime;
    int32_t actualRelativeTimeDifference = actualTimeDifference - _interval;
    if (resetOnTrigger && actualRelativeTimeDifference >= 0){
        resetTimingNow (nowMs);
    }
    return actualRelativeTimeDifference;
}
