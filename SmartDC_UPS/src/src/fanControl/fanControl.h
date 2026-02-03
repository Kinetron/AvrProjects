#include <Arduino.h>

#define FAN_PWM_PIN 10

// PWM Speed settings
#define SPEED_MIN 80           // Minimum PWM to keep the motor spinning reliably.
#define SPEED_MAX 255          // Maximum PWM (100% duty cycle).
#define KICK_MS 300            // Duration of the start-up "kick" in milliseconds.
#define FAN_REACTION_TIME 5000 // 5sec

// Thresholds in millivolts (or raw ADC units).
#define TEMP_ON 3260UL  // Fan starts at this value (e.g., 40°C).
#define TEMP_OFF 3150UL // Fan stops at this value (Hysteresis, ~38.5°C).
#define TEMP_MAX 4000UL // Fan reaches 100% speed at this value (60°C).

// Sets up fan GPIO pins.
void fanInit();

// Controls the fan speed.
void fanControl(uint32_t sensorVoltage);