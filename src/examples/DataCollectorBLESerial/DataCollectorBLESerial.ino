//1. Load this sketch and download Serial Bluetooth Terminal. 
//  - Android (https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en_GB&gl=US)
//  - App Store (https://apps.apple.com/gb/app/bluetooth-terminal/id1058693037)- not tested
//2. Pair with the device in the phone settings then select it on the App. 
//3. Connect to the phone and data should be streaming
#include <imu.h>
#include <timeOfFlight.h>
#include "BluetoothSerial.h"

// Forward declaration
void uvloProtection();
void run_inference();
void sampleToBuffer();

float lowVoltageThreshold = 2.7; // set low voltage threshold to turn off peripherals and set MCU to deep sleep when battery voltage is at 2.7V
float updateRate = 50; //in Hz
BluetoothSerial SerialBT;
IMU myIMU;

/* Private variables ------------------------------------------------------- */

int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;

float battVoltage = 0;
int timeStamp = 0;



void setup() {
  battVoltage = 4.15 * analogRead(35) * (3.3 / 4095.0); // convert ADC value to voltage
  //  Check if battery voltage is ok
  uvloProtection();
  
  Serial.begin(115200);
  SerialBT.begin("ESP32-Tracer"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(35, INPUT); //  Set battery voltage monitoring
  myIMU.init();
}



void loop()
{
    t_0 = millis();
    uvloProtection();
    battVoltage = 4.25 * analogRead(35) * (3.3 / 4095.0);
    myIMU.update();
    ax = myIMU.readAccel('x');
    ay = myIMU.readAccel('y');
    az = myIMU.readAccel('z');
    gx = myIMU.readGyro('x');
    gy = myIMU.readGyro('y');
    gz = myIMU.readGyro('z');
    String msg = "ax : " + String(ax) + ", ay : " + String(ay) + ", az : " + String(az) + ", gx : " + String(gx) + ", gy : " + String(gy) + ", gz : " + String(gz);
    SerialBT.println(msg);

    t_1 = millis();

    // Check if value is not negative - update rate is achievable
    if ((1000 / updateRate - (t_1 - t_0)) > 0)
    {
      delay(1000 / updateRate - (t_1 - t_0));
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
};
