#include <imu.h>
#include <timeOfFlight.h>
#include <bluetooth.h>
#include <phyphoxBle.h>
#include <MadgwickAHRS.h>
#include "get-on-pedals-with-arduino-nano_inferencing.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  4.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

void uvloProtection();

float lowVoltageThreshold = 2.7; // set low voltage threshold to turn off peripherals and set MCU to deep sleep when battery voltage is at 2.7V
float updateRate = 25; //in Hz

float ax, ay, az, gx, gy, gz;
float roll, pitch, heading;
int distanceToF;
float count = 0;
int timeOfHit = 0;
bool timedOut = true;
float battVoltage = 0;
Bluetooth ble;
IMU myIMU;
//TOF myTOF;

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

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


/**
   @brief Return the sign of the number

   @param number
   @return int 1 if positive (or 0) -1 if negative
*/
float ei_get_sign(float number) {
  return (number >= 0.0) ? 1.0 : -1.0;
}

void loop()
{
  ei_printf("\nStarting inferencing in 2 seconds...\n");

  delay(2000);

  ei_printf("Sampling...\n");

  // Allocate a buffer here for the values we'll read from the IMU
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
    // Determine the next tick (and then sleep later)
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
    myIMU.update();
    ax = myIMU.readAccel('x');
    ay = myIMU.readAccel('y');
    az = myIMU.readAccel('z');
    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = ax;
    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = ay;
    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = az;

    for (int i = 0; i < 3; i++) {
      if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
        buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
      }
    }

    buffer[ix + 0] *= CONVERT_G_TO_MS2;
    buffer[ix + 1] *= CONVERT_G_TO_MS2;
    buffer[ix + 2] *= CONVERT_G_TO_MS2;

    delayMicroseconds(next_tick - micros());
  }

  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
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
