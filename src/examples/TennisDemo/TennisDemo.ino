#include <imu.h>
#include <timeOfFlight.h>
#include <bluetooth.h>
#include <phyphoxBle.h>
#include <tracer-tennis_stroke_inferencing.h>
/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  4.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

// Forward declaration
void uvloProtection();
void run_inference();
void sampleToBuffer();

float lowVoltageThreshold = 2.7; // set low voltage threshold to turn off peripherals and set MCU to deep sleep when battery voltage is at 2.7V
float updateRate = 50; //in Hz
bool dataCollector = false;

Bluetooth ble;
IMU myIMU;

/* Private variables ------------------------------------------------------- */
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
int t_0;
int t_1;
float ax, ay, az, gx, gy, gz;
float count = 0;
int timeOfHit = 0;
bool timedOut = true;
float battVoltage = 0;
int timeStamp = 0;
int noTsHits = 0;
int noSlHits = 0;
int noFlatHits = 0;


void setup() {
  battVoltage = 4.15 * analogRead(35) * (3.3 / 4095.0); // convert ADC value to voltage
  //  Check if battery voltage is ok
  uvloProtection();
  Serial.begin(115200);

  pinMode(35, INPUT); //  Set battery voltage monitoring
  myIMU.init();
  //  myTOF.init();
  ble.init(dataCollector);
}

/**
   @brief Return the sign of the number

   @param number
   @return int 1 if positive (or 0) -1 if negative
*/
float ei_get_sign(float number) {
  return (number >= 0.0) ? 1.0 : -1.0;
}

/**
   @brief      Run inferencing in the background.
*/
void run_inference()
{
  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }


  // Run the classifier
  ei_impulse_result_t result = { 0 };
  err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

//  // print the predictions
//  ei_printf("Predictions ");
//  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
//            result.timing.dsp, result.timing.classification, result.timing.anomaly);
//  ei_printf(": \n");
//  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
//  }

  if (result.classification[0].value > (result.classification[1].value + 0.25))
  {
    noSlHits++;
    Serial.println("Slice");
  }
  else if (result.classification[1].value > (result.classification[0].value + 0.25))
  {
    noTsHits++;
    Serial.println("Top Spin");
    
  }
  else 
  {
    noFlatHits++;
    Serial.println("Flat");
  }
}



void loop()
{
  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
    t_0 = millis();
    uvloProtection();
    battVoltage = 4.25 * analogRead(35) * (3.3 / 4095.0);

    sampleToBuffer();

    /* Simple tennis shot counter*/
    // if no hit event for > 0.3s, change timeout flag to false
    if (millis() > (timeOfHit + 300)) // wait 300ms between hits
    {
      timedOut = true;
    }
    // if hit event happened
    if (abs(az) > 3)
    {
      timeOfHit = millis();
      // if timed out and hit event registed, increase count
      if (timedOut)
      {
        // Sample for another 300ms after hit is registered
        while (millis() - timeOfHit < 300)
        {
          int tStart = millis();
          sampleToBuffer();
          delay(1000 / updateRate - (millis() - tStart)); // fix to sample of the same rate
        }

        if (dataCollector)
        {
          // Send windowed data here for the window collected at
          for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            int tStart = millis();
            // Send data to Phyphox
            ble.update(timeStamp, buffer[ix], buffer[ix + 1], buffer[ix + 2]);
            timeStamp = timeStamp + 1000 / updateRate;
            delay(1000 / updateRate - (millis() - tStart)); // fix to sample of the same rate
          }
        }
        count++;
        timedOut = false;
      }

    }

    if (!timedOut && !dataCollector)
    {
      run_inference();
      ble.update(count, noTsHits, noSlHits, noFlatHits);

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
};

void sampleToBuffer()
{
  /*  IMU Update  */
  myIMU.update();
  ax = myIMU.readAccel('x');
  ay = myIMU.readAccel('y');
  az = myIMU.readAccel('z');
  // roll the buffer -3 points so we can overwrite the last one
  numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = ax;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = ay;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = az;
  for (int i = 0; i < 3; i++) {
    if (fabs(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) > MAX_ACCEPTED_RANGE) {
      buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] = ei_get_sign(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) * MAX_ACCEPTED_RANGE;
    }
  }
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] *= CONVERT_G_TO_MS2;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] *= CONVERT_G_TO_MS2;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] *= CONVERT_G_TO_MS2;
};


//  IMU Reference Code
/*
   https://docs.arduino.cc/library-examples/curie-imu/Genuino101CurieIMUOrientationVisualiser
   https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
   https://github.com/xioTechnologies/Fusion/blob/master/Examples/ExampleAhrsWithoutMagnetometer.c
*/
