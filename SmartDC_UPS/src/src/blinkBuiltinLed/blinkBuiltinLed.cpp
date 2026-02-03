#include "blinkBuiltinLed.h"

bool blinkBuiltinLedOn;
uint32_t blinkIntervalBuiltin;
uint32_t lastBlinkTimeBuiltin;

void blinkBuiltinLedInit()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

// On/Off  blink built-in led on the arduino nano board.
void blinkBuiltinLed(bool on, uint8_t blinkInterval)
{
  blinkBuiltinLedOn = on;
  blinkIntervalBuiltin = blinkInterval;
}

void offBlinkBuiltinLed()
{
  blinkBuiltinLedOn = false;
}

// Blink built-in led on the arduino nano board.
void blinkBuiltinLedProcess()
{
  if (!blinkBuiltinLedOn)
  {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  uint32_t currentTime = millis();

  if (currentTime - lastBlinkTimeBuiltin >= blinkIntervalBuiltin)
  {
    lastBlinkTimeBuiltin = currentTime;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}