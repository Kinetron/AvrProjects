#include "fanControl.h"

uint32_t fanLastLogicTime;
uint8_t fanCurrentSpeed;
uint32_t fanKickstopTime;

// Sets up fan GPIO pins.
void fanInit()
{
  pinMode(FAN_PWM_PIN, OUTPUT);
  digitalWrite(FAN_PWM_PIN, HIGH); // Off fan.
}

// Controls the fan speed.
void fanControl(uint32_t sensorVoltage)
{
  uint32_t now = millis();

  // Run fan control logic every x seconds.
  if (now - fanLastLogicTime < FAN_REACTION_TIME)
  {
    return;
  }

  fanLastLogicTime = now;

  // 2414mv at 24 degrees

  uint8_t targetSpeed = 0;

  // Hysteresis logic
  if (sensorVoltage > TEMP_ON)
  {
    // Map the input value to PWM range.
    int32_t mapped = map(sensorVoltage, TEMP_ON, TEMP_MAX, SPEED_MIN, SPEED_MAX);
    targetSpeed = (uint8_t)constrain(mapped, SPEED_MIN, SPEED_MAX);
  }
  else if (sensorVoltage < TEMP_OFF)
  {
    // Turn off if below the lower threshold.
    targetSpeed = 0;
  }
  else
  {
    // Keep previous state if value is within the "dead zone" (hysteresis).
    targetSpeed = fanCurrentSpeed;
  }

  // Check if we need to start a kickstart burst
  if (targetSpeed > 0 && fanCurrentSpeed == 0)
  {
    fanKickstopTime = millis() + KICK_MS;
  }

  fanCurrentSpeed = targetSpeed;

  if (millis() < fanKickstopTime)
  {
    analogWrite(FAN_PWM_PIN, 0); // Max speed.
  }
  else
  {
    analogWrite(FAN_PWM_PIN, SPEED_MAX - fanCurrentSpeed);
  }
}
