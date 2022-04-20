#include "timeOfFlight.h"


TOF::TOF()
{

};

void TOF::init()
{
  Wire.begin(18, 23);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    //    while (1) {}
  }
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
};



void TOF::update()
{
  distance = sensor.readRangeSingleMillimeters();
};

int TOF::readDistance()
{
  return distance;
}
TOF::~TOF()
{

};
