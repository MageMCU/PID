
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
    controlManager = pid::ControlManager<float>((float)0.05);
    // controlManager = pid::ControlManager<float>((float)0, (float)1000, (float)0.05);
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
        while (ctr < 50)
        {
            controlManager.UpdateControlManager();

            // Debug - Total of 10-columns - csv-file
            // Serial.print(ctr); // Number of Data Points
            // Serial.print(",");
            Serial.print(controlManager.PrintSetPoint()); // No change
            Serial.print(",");
            Serial.print(controlManager.PrintMeasuredValue()); // Approaches SetPoint
            Serial.print(",");
            Serial.print(controlManager.PrintControlPID()); // Control approaches zero
            // Serial.print(",");
            // Serial.print(controlManager.PrintEID_PID()); // e, i, d - Interesting to watch
            // Serial.print(",");
            // Serial.print(controlManager.PrintFunctionsPID()); // Fp, Fi, Fd - Individual Functions
            Serial.println();

            ctr++;
        }


    }
}