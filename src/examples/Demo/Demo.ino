#include <imu.h>
#include <bluetooth.h>
#include <phyphoxBle.h>
#include <MadgwickAHRS.h>


void uvloProtection();

float lowVoltageThreshold = 2.7; // set low voltage threshold to turn off peripherals and set MCU to deep sleep when battery voltage is at 2.7V
float updateRate = 10; //in Hz
int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;
float absolute_a;
float roll, pitch, heading;
int distanceToF;
float count = 0;
int timeOfHit = 0;
bool timedOut = true;
float battVoltage = 0;
Bluetooth ble;
IMU myIMU;


Madgwick filter;

void setup() {
  battVoltage = 4.15 * analogRead(35) * (3.3 / 4095.0); // convert ADC value to voltage
  //  Check if battery voltage is ok
  uvloProtection();
  Serial.begin(115200);

  pinMode(35, INPUT); //  Set battery voltage monitoring
  myIMU.init();
  //  myTOF.init();
  ble.init();
  filter.begin(updateRate);
}


void loop()
{
  t_0 = millis();
  uvloProtection();
  battVoltage = 4.25 * analogRead(35) * (3.3 / 4095.0);
  //  Serial.println(battVoltage);
  /*  TOF update  */
  //  myTOF.update();
  //  distanceToF = myTOF.readDistance();

  /*  IMU Update  */
  myIMU.update();
  ax = myIMU.readAccel('x');
  ay = myIMU.readAccel('y');
  az = myIMU.readAccel('z');
  gx = myIMU.readGyro('x');
  gy = myIMU.readGyro('y');
  gz = myIMU.readGyro('z');

  /*  Madgwick Filter update  */
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  /* Simple tennis shot counter*/
  // if no hit event for > 0.5s, change timeout flag to false
  if (millis() > (timeOfHit + 300)) // wait 500ms between hits
  {
    timedOut = true;
    // if hit event happened
    if (abs(az) > 3.5)
    {
      timeOfHit = millis();
      // if timed out and hit event registed, increase count
      if (timedOut)
      {
        count++;
        timedOut = false;
      }
    }

    /* Send data packet to Tracer

       Ping data only when hit event has occured - 500ms to track what is happening after hit
    */
   if (!timedOut)
   {
    ble.update(ax, ay, az, count);
   }

    t_1 = millis();

    // Check if value is not negative - update rate is achievable
    if ((1000 / updateRate - (t_1 - t_0)) > 0)
    {
      delay(1000 / updateRate - (t_1 - t_0));
    }
  }
}

void uvloProtection()
{
  /* This is a software low voltage protection that sets the ToF and IMU to the lowest current setting and puts the MCU to Hibernation mode.

  */
  if (battVoltage < lowVoltageThreshold)
  {
    myIMU.end();
    digitalWrite(19, LOW);
    delay(50);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }
}
//  IMU Reference Code
/*
   https://docs.arduino.cc/library-examples/curie-imu/Genuino101CurieIMUOrientationVisualiser
   https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
   https://github.com/xioTechnologies/Fusion/blob/master/Examples/ExampleAhrsWithoutMagnetometer.c
*/
