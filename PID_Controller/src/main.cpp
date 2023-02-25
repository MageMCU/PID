//
// Carpenter Software
// File: main.cpp
//
// Purpose: Public Github Account - MageMCU
// Repository: PID-Controller 
// Date Created: 20230215
// Folder: PID_Controller/src
//
// Author: Jesse Carpenter (carpentersoftware.com)
// Email:carpenterhesse@gmail.com
//
// Testing Platform:
//  * MCU:Atmega328P
//  * Editor: VSCode
//  * VSCode Extension: Microsoft C/C++ IntelliSense, debugging, and code browsing.
//  * VSCode Extension:PlatformIO
// 
// Revised Header-Comment 20230219
//
// MIT LICENSE
//

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

    float min = (float)0;
    float max = (float)1000;
    float  Kp = (float)0.95;
    float  Ki = (float)0.0;
    float  Kd = (float)0.0;
    float  Ts = (float)0.050; // Sampled-Interval Ts

    // Sampled Time is 50 ms (or 0.05 s)
    controlManager = pid::ControlManager<float>(Kp, Ki, Kd, Ts, min, max);
    controlTimer = nmr::Timer();

    // Locate beinning csv-data
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