#include <Arduino.h>

#define BLINK_INTERVAL_1S 1000
#define BLINK_INTERVAL_03S 150

void blinkBuiltinLedInit();

// Blink built-in led on the arduino nano board.
void blinkBuiltinLedProcess();

// On/Off  blink built-in led on the arduino nano board.
void blinkBuiltinLed(bool on, uint8_t blinkInterval);

void offBlinkBuiltinLed();