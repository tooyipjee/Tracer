/******************************************************************************
bluetooth.h
Phyphox bluetooth wrapper

Jason Too
March 25, 2022

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact hello@jasontoo.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include <phyphoxBle.h>

class Bluetooth
{
  public:
    Bluetooth();
    ~Bluetooth();
    void init();
    void update(float f0, float f1, float f2, float f3, float f4);
    void update(float f0, float f1, float f2, float f3);
    void update(float f0, float f1, float f2);
    void update(float f0, float f1);
    void update(float f0);
    float readInput = 0;
    float editCount = 0;
};

#endif
