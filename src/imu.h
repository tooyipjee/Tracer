/******************************************************************************
imu.h
LSM6DSL  wrapper

elektrothing
March 25, 2022

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact hello@jasontoo.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef IMU_H
#define IMU_H

#include "elektroThing_LSM6DSL.h"

class IMU
{
  public:
    IMU();
    ~IMU();
    void init();
    void update();
    float readAccel(char input);
    float readGyro(char input);
    float readTemp();
    void end();
    
  private:
    LSM6DS3 myIMU;

    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float temperature;
};

#endif
