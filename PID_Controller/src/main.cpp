
#include "Arduino.h"

#include "ControlManager.h"
#include "Timer.h"

pid::ControlManager<float> controlManager;
nmr::Timer controlTimer;

// Comment ......................
void setup()
{
    Serial.begin(9600);
    while (!Serial) {}

    // Sampled Time is 50 ms (or 0.05 s)
    controlManager = pid::ControlManager<float>((float)0, (float)1000, (float)0.05);
    controlTimer = nmr::Timer();

    Serial.println(".");
    Serial.println(".");
    Serial.println(".");
}

int ctr = 0;
void loop()
{
    // Under Develoment
    if (controlTimer.isTimer(50))
    {
        while (ctr < 25)
        {
            controlManager.UpdateControlManager();

            // Debug

            Serial.print(ctr);
            Serial.print(",");
            Serial.print(controlManager.PrintFunctionsPID());
            Serial.print(",");
            Serial.print(controlManager.PrintSetPoint());
            Serial.print(",");
            Serial.println(controlManager.PrintMeasuredValue());

            ctr++;
        }


    }
}