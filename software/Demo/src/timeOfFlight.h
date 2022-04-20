/******************************************************************************
timeOfFlight.h
VL53L0X  wrapper

Jason Too
April 20, 2022

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact hello@jasontoo.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef TOF_H
#define TOF_H

#include "elektroThing_VL53L0X.h"
//#define LONG_RANGE
class TOF
{
  public:
    TOF();
    ~TOF();
    void init();
    void update();
    int readDistance();    
  private:
    VL53L0X sensor;
    int distance;
};

#endif
