#include "application.h"

Application::Application(){}
Application::~Application(){}

void Application::init(int state, int imuSampleRate) 
{
  imuSampleRate = imuSampleRate;

  // Set GPIO type
  pinMode(IMU_PWR, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  pinMode(SET, INPUT);
  pinMode(BATT, OUTPUT);
  pinMode(READ_BATT, INPUT);
  // Turn on the IMU
  digitalWrite(IMU_PWR, HIGH);
  // Allow time for power lines to settle
  delay(100);
  // IMU
  myIMU = new LSM6DS3;
  // Example of setting the IMU parameters
  myIMU -> settings.accelSampleRate = 52;
  myIMU->begin();

  // Depending on which state, chose the appropriate BLE stack
  switch (state)
  {
    case 0:
    // NO BLE INIT
      Serial.println("No BLE Stack");
      break;
    case 1:
      Serial.println("Setting to Phyphox BLE");
      // Init Phyphox BLE 
      pBle.init();
      break;
    case 2:
      Serial.println("Setting to BLE5 advertising");
      // setupBluetooth();
      break;

  }

}

void Application::update(int state) 
{

  switch (state)
  {
    case 0:
      LogOverUART();
      break;
    case 1:
      LogOverBLE();
      break;
    case 2:
      Inference();
      break;
    case 3:
    // Do nothing - allows serial port to be accessed
      toggleLed(15);      
      break;
  }


}

void Application::sampleIMU() // in Hz
{
  // If current time is larger than the last sample plus period - get a new sample
  if (millis() > imuSampleTime + 1000/imuSampleRate)
  {
    AccX = myIMU->readFloatAccelX();
    AccY = myIMU->readFloatAccelY();
    AccZ = myIMU->readFloatAccelZ();
    GyX = myIMU->readFloatGyroX();
    GyY = myIMU->readFloatGyroY();
    GyZ = myIMU->readFloatGyroZ();
    imuSampleTime = millis();
  }
}

void Application::LogOverUART()
{
  toggleLed(3);

  // Dump readings to UART with the following format -> ax,ay,az, gx, gy, gz
  // Accelerometer measurements
  Serial.print(AccX, 4);
  Serial.print(",");
  Serial.print(AccY, 4);
  Serial.print(",");
  Serial.print(AccZ, 4);
  // Gyro measurements
  Serial.print(",");
  Serial.print(GyX, 4);
  Serial.print(",");
  Serial.print(GyY, 4);
  Serial.print(",");
  Serial.println(GyZ, 4);
  
  
}

void Application::LogOverBLE()
{
  toggleLed(2);

  pBle.update(AccX, AccY, AccZ, GyX, GyY);

}

void Application::Inference()
{
  toggleLed(1);

  // This application is designed to run with minimal logging to extend the battery life as much as possible.



}

void Application::toggleLed(int numberOfBlinks)
{
  if (millis() > ledBlinkTime + (numTimesToggled)*300 + 5000)
  {
    if(ledStatus)
    {
      digitalWrite(DEBUG_LED, HIGH);
    }
    else
    {
      digitalWrite(DEBUG_LED, LOW);
    }
    ledStatus = !ledStatus;
    numTimesToggled++;
  }

  if (numTimesToggled >= 2*numberOfBlinks)
  {
    ledBlinkTime = millis();
    numTimesToggled = 0;
  }
}