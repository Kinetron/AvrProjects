#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>
#include "lcd/lcd.h"
#include "fanControl/fanControl.h"
#include "buttons/buttons.h"
#include "blinkBuiltinLed/blinkBuiltinLed.h"
#include "types.h"

// Line voltage sensor.
#define LINE_SENSE_PIN 2
#define LINE_TIMEOUT_MS 25

// Pins for relays, which switch the power sources for load.
#define RELAY_PSU_PIN 4
#define RELAY_BAT_PIN 3

#define MAIN_LOAD_ON_PIN 9
#define SLAVE_LOAD_ON_PIN 12
#define SHUTDOWN_TIME_MAIN_LOAD_MOSFET 5
#define RELAY_SWITCH_TIME 25

#define RELAY_STEPDOWN_SWITCH_SOURCE_PIN 8
#define RELAY_FULL_CHARGE_CURRENTPIN 7

#define TURN_ON_FULL_CHARGE_PIN 7
#define FULL_CHARGE_DELAY 10
#define MIN_THRESHOLD_BATTERY_FULL_CHARGE_ON 23100

// If the line is disconnected, the main and secondary loads are battery powered.
// But the secondary load will shut down after a specified period of time.
#define OFF_TIME_SECOND_LOAD 5

#define TEMPERATURE_SENSOR_VOLTAGE_INDEX 4

// Delay after power on in seconds.
#define STARTUP_DELAY 5
#define LINE_ON_DELAY 7 // Stabilization interval after line voltage recovery.
// If you switched to on mode after the line appeared.
// But the line started disappearing/reappearing. If it has not stabilized, we exit this mode.
#define LINE_ON_ATTEMPTS 10

#define MAIN_UPPER_THRESHOLD 12800UL      // 12.5V in mV
#define MAIN_DOWN_LOWER_THRESHOLD 11500UL // 11.5V in mV
#define MAIN_VOLTAGE_INDEX 0

#define STEP_DOWN_UPPER_THRESHOLD 12800UL // 12.5V in mV
#define STEP_DOWN_LOWER_THRESHOLD 11500UL // 11.5V in mV
#define STEP_DOWN_VOLTAGE_INDEX 1

#define BATTERY_LOWER_THRESHOLD 10100UL
#define BATTERY_VOLTAGE_INDEX 2

#define LOAD_CURRENT_INDEX 3
// Delay for obtaining ADC values before calibrating the sensor. Seconds.
#define ACS712_CALIBRATE_TIMER_DELAY 3

#define V_REF_MV 5000.0f
#define ADC_RES 1024.0f
#define SAMPLES_SHIFT 16 // 1024 (ADC_RES) * 64 (SAMPLES_COUNT) = 65 536, which is 2^16
#define SAMPLES_COUNT 64 // Explicit 64 samples
#define NUM_CHANNELS 5

// Resistor Values (1% precision).
#define R_10K 10000.0f
#define R_5K1 5100.0f
#define RATIO_A2 6.652f

#define CALC_DIVIDER_COEFF(rh, rl) (uint32_t)(((rh + rl) / rl) * V_REF_MV)
#define CALC_DIRECT_COEFF(ratio) (uint32_t)(ratio * V_REF_MV)

#define GET_ADC_COEFF(r_high, r_low) (uint32_t)(((r_high + r_low) / r_low) * V_REF_MV / ADC_RESOLUTION)
#define GET_DIRECT_COEFF(ratio) (uint32_t)(ratio * V_REF_MV / ADC_RESOLUTION)

const uint8_t ADC_PINS[NUM_CHANNELS] = {0, 1, 2, 3, 7};

const uint32_t ADC_COEFFS[NUM_CHANNELS] = {
    CALC_DIVIDER_COEFF(R_10K, R_5K1), // A0 (12.5V)
    CALC_DIVIDER_COEFF(R_10K, R_5K1), // A1 (12.5V)
    CALC_DIRECT_COEFF(RATIO_A2),      // A2 (max32V) 43k + 7.5k
    CALC_DIRECT_COEFF(1.0),           // A3 (ACS712 direct input, no divider)
    CALC_DIRECT_COEFF(1.0)            // A7 Thermistor 10k
};

// All internal variables required for the module to work.
struct SystemParameters
{
  OperationMode mode;
  bool lineOn;
  uint32_t lineOnTimer;
  uint32_t attemptsAfterlineOn;
  uint8_t waitingTimePassed;
  bool OffMainLoad;
  bool OffSlaveLoad; 
  bool OffSlaveLoadFromBtn; // Off load with the button.
  bool lastPowerState; // Was the load enabled or disabled last time.
  uint32_t onFullChargeTimer;
  bool fullChargeOn;
  uint32_t ACS712calibrateTimer;
  bool isCalibrateACS712;
  uint32_t ACS712zeroOffset;
  uint32_t periodOffSecondLoadTimer;
  bool lastOffSecondLoadStatus;
};

enum class LoadSource : uint8_t
{
  None,    // Load is disconnected.
  MainPSU, // Load powered by Main Power Supply Unit (Line).
  Battery  // Load powered by Battery.
};

// Returns the number of seconds since the start.
uint32_t getSeconds();

void logic_init();

void switchModes();

// The function that we will call in the loop.
void logic_tick();

// Has there been a delay after turning on the device?
bool initDelayPassed();

void startupStep();

void workAfterload();

// Switch power sources.
void switchPowerSources(LoadSource sourceType);

// Connect the load to an power source.
void loadOn(LoadSource sourceType);

// Delay after AC line recovery.
bool lineOnTimer();

// Reset line on timer.
void resetlineOnTimer();

// Timer for waiting for stable line voltage.
bool attemptsAfterlineOnTimer();

// Reset timer for waiting for stable line voltage.
void resetAttemptsAfterlineOnTimer();

// The waiting step after the voltage appears in the line.
void lineAfterBatteryStep();

void readAdc();

// Rising pulse line voltage. Every 10ms.
void line_pulse_isr();

// Сheck line voltage.
void checkLine();

bool checkMainDcVoltage();

// Checks the output voltage step down.
bool checkStepDownVoltage();

void protectOvervoltage();

// Monitors the moment when the line voltage is restored.
void monitorLineRecovery();

// Shows the output voltage and current on the display.
void showOutputParams(LoadSource sourceType);

// Controls the battery charge current.
void chargelogic();

// Full charge interval timer.
bool fullChargeTimer();

// Resets the full charge timer.
void resetFullChargeTimer();

// Calibrates the ACS712 current sensor.
void calibrateACS712();

// Checks if the ACS712 calibration interval has elapsed.
bool ACS712calibrateTimer();

// Resets the timer for ACS712 sensor calibration.
void resetACS712calibrateTimer();

// Turns off the secondary load after a certain time after the line drops.
void autoOffSecondLoad();

// Resets the second load outage interval timer.
bool offSecondLoadTimer();

// Resets the second load outage timer.
void resetOffSecondLoadTimer();

#endif
