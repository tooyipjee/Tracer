#include <imu.h>
#include <timeOfFlight.h>
#include <bluetooth.h> 
float updateRate = 100; //in Hz
int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;
int distanceToF;

Bluetooth ble;
IMU myIMU;
TOF myTOF;

void setup() {

  Serial.begin(115200);

  myIMU.init();
  myTOF.init(1);
  ble.init();

  Serial.println("d,ax,ay,az,gx,gy,gz");
}


void loop()
{

//  /*  TOF update  */
//  myTOF.update();
//  distanceToF = myTOF.readDistance();
//  Serial.print(distanceToF); Serial.print(",");

  /*  IMU Update  */
  myIMU.update();
  ax = myIMU.readAccel('x');
  ay = myIMU.readAccel('y');
  az = myIMU.readAccel('z');
  gx = myIMU.readGyro('x');
  gy = myIMU.readGyro('y');
  gz = myIMU.readGyro('z');


  ble.update(ax,ay,az,gx,gy);

  t_1 = millis();

  // Check if value is not negative - update rate is achievable
  if ((1000 / updateRate - (t_1 - t_0)) > 0)
  {
    delay(1000 / updateRate - (t_1 - t_0));
  }
}
