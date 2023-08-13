#ifndef APPLICATION_H
#define APPLICATION_H
#include "elektroThingLSM6DSL.h"
#include "bluetooth.h"
#include "hardwareConfig.h"
// #include "bluetoothAdv.h"

typedef enum 
{
  LOG_UART = 0,
  LOG_BLE  = 1,
  INFERENCE  = 2,
} States;

class Application
{
  public: 
    Application();
    ~Application();
    void init(int state, int imuSampleRate);
    void update(int state);
    void sampleIMU();
    void LogOverUART();
    void LogOverBLE();
    void Inference();
    void toggleLed(int numberOfBlinks);
  private:
    int state = 0;

    LSM6DS3* myIMU; //Default constructor is I2C, addr 0x6B
    Bluetooth pBle;

    // IMU Settings
    float AccX, AccY, AccZ, GyX, GyY, GyZ;
    int imuSampleTime = 0;
    int imuSampleRate = 10; // if setting > 100, please change the value in elektroThingLSM6DSL line 353, or call

    // LED
    int ledBlinkTime = 0;
    bool ledStatus = true;
    int numTimesToggled = 0;

};
#endif