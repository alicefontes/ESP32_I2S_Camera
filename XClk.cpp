#include "XClk.h"

#include <Arduino.h>

static int xclkPin = -1;

bool ClockEnable(int pin, int Hz)
{
    xclkPin = pin;

    // Arduino-ESP32 Core 2.x
    ledcSetup(0, Hz, 1);
    ledcAttachPin(pin, 0);

    // Duty cycle de 50%
    ledcWrite(0, 1);

    return true;
}

// void ClockDisable()
// {
//     if (xclkPin >= 0)
//     {
//         ledcDetachPin(xclkPin);
//         xclkPin = -1;
//     }
// }
void ClockDisable()
{
    if (xclkPin >= 0) {

        ledcWrite(0, 0);
        ledcDetachPin(xclkPin);

        pinMode(xclkPin, OUTPUT);
        digitalWrite(xclkPin, LOW);

        xclkPin = -1;
    }
}