#include <imu.h>
#include <timeOfFlight.h>

float updateRate = 100; //in Hz
int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;
int distanceToF;

#define G_TO_MS2 9.81

IMU myIMU;
TOF myTOF;

void setup() {

  Serial.begin(115200);

  myIMU.init();
  myTOF.init(1);

  Serial.println("ax,ay,az,gx,gy,gz");
}


void loop()
{
  t_0 = millis();
  /*  TOF update  */
//  myTOF.update();
//  distanceToF = myTOF.readDistance();
//  Serial.print(distanceToF); Serial.print(",");
  
  /*  IMU Update  */
  myIMU.update();
  ax = myIMU.readAccel('x')*G_TO_MS2;
  ay = myIMU.readAccel('y')*G_TO_MS2;
  az = myIMU.readAccel('z')*G_TO_MS2;
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
    delay(1000 / updateRate - (millis() - t_0));
  }
  else
  {
    Serial.println("Loop running slower than set frequency");
  }
}
