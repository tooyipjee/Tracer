#include "application.h"

RTC_NOINIT_ATTR unsigned int state = 0;

#define   PRINT_IMU   1

Application app;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  app.init(state, 100);
  Serial.println("Processor came out of reset.\n");
}


void loop()
{
  // This loop must not have any delays.
  if (digitalRead(SET) == 0)
  {
    state++;
    if(state>3)
    {
      state = 0;
    }
    delay(500);
    ESP.restart();
  };

  app.update(state);
  app.sampleIMU();
}