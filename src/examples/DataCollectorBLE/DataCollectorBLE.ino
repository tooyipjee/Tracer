#include <imu.h>
#include <timeOfFlight.h>

float updateRate = 1000; //in Hz
int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;
int distanceToF;


IMU myIMU;
TOF myTOF;

void setup() {

  Serial.begin(115200);

  myIMU.init();
  myTOF.init(1);

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

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  t_1 = millis();

  // Check if value is not negative - update rate is achievable
  if ((1000 / updateRate - (t_1 - t_0)) > 0)
  {
    delay(1000 / updateRate - (t_1 - t_0));
  }
}
