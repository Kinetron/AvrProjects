#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "types.h"

#define LCD_INTERVAL 700 // 0.7 second.

// Battery voltage limits in millivolts (mV) for display the charge level.
#define BATTERY_MIN_MV 21000UL // 0% charge
#define BATTERY_MAX_MV 28600UL // 100%

// ACS712 5A Model Constants
// 2500mV is the theoretical mid-point (Zero Current) for a 5V supply
#define ACS_OFFSET 2500

// Sensitivity for 5A version is 185mV per 1 Ampere
#define ACS_SENSITIVITY 185

// Structure to store decomposed voltage for display purposes.
struct VoltageDisplay
{
  uint16_t whole;   // Integer part of the voltage
  uint16_t decimal; // First decimal place (tenth)
};

// Defines the required precision for decimal extraction.
enum class Precision : uint8_t
{
  Tenths = 100,   // (val % 1000) / 100 -> gives 1 digit (0-9)
  Hundredths = 10 // (val % 1000) / 10  -> gives 2 digits (00-99)
};

// Initializes the LCD display.
void initLCD();

// Displays messages and voltage on the screen.
void displayService(OperationMode mode, uint32_t main, uint32_t stepDown, uint32_t battery, 
                uint32_t adcCurrent, uint32_t currentZeroOffset);

// Adds spaces. Padding the string to 16 characters.
void addSpaces();

// Starts the process of displaying messages on the screen.
void printMessage(const char *text);

// Displays voltages and currents on the display.
void showParams(OperationMode mode, uint32_t main, uint32_t stepDown, uint32_t battery, 
                uint32_t current, uint8_t chargeLevel);

// Split raw voltage values (in mV or scaled units).
VoltageDisplay parseVoltage(uint32_t rawValue, Precision type);

// Displays voltages of main dc, output step down, battery.
void showVoltages(uint32_t mainV, uint32_t stepDownV, uint32_t batteryV);

// Displays the output parameters on the LCD second line.
void showOutputParams(bool isBatteryMode, uint8_t chargeLevel,
     uint32_t current);

// Calculates the battery charge level in percent.
uint8_t batteryChargeLevel(uint32_t voltage_mv);

// Calculates the load current in milliamperes (mA).
uint32_t calculateACS712Current(uint32_t adcValue, uint32_t zeroOffset);

// Calculating the line outage time.
bool lineOffClock();

// Resets the line outage clock.
void lineOffClockReset();