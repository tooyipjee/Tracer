#include "imu.h"


IMU::IMU()
{

};

void IMU::init()
{
  //Call .begin() to configure the IMU
  myIMU.begin();
};

void IMU::update()
{
  accel_x = myIMU.readFloatAccelX();
  accel_y = myIMU.readFloatAccelY();
  accel_z = myIMU.readFloatAccelZ();

  gyro_x = myIMU.readFloatGyroX();
  gyro_y = myIMU.readFloatGyroY();
  gyro_z = myIMU.readFloatGyroZ();

  temperature = myIMU.readTempC();
};

float IMU::readAccel(char input)
{
  if (input == 'x')
  {
    return accel_x;
  }
  else if (input == 'y')
  {
    return accel_y;
  }
  else if (input == 'z')
  {
    return accel_z;
  }
  return 0;
}

float IMU::readGyro(char input)
{
  if (input == 'x')
  {
    return gyro_x;
  }
  else if (input == 'y')
  { 
    return gyro_y;
  }
  else if (input == 'z')
  {
    return gyro_z;
  }
  return 0;
}

float IMU::readTemp()
{
  return temperature;
}

IMU::~IMU()
{

};
