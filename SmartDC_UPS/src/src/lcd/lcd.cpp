#include "lcd.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
bool isNeedDisplayed;
uint32_t lastLcdUpdate;
char displayBuffer[17];
char previousDisplayBuffer[17];
TimeValues linePowerOffClock;
uint32_t lastClockTick;
uint32_t lineOffStartTime;

// Initializes the LCD display.
void initLCD()
{
  lcd.init();
  lcd.backlight();
}

// Displays messages and voltage on the screen.
void displayService(OperationMode mode, uint32_t main, uint32_t stepDown, uint32_t battery,
                    uint32_t adcCurrent, uint32_t currentZeroOffset)
{
  // Print text if change.
  if (isNeedDisplayed)
  {
    addSpaces();
    lcd.setCursor(0, 0);
    lcd.print(displayBuffer);
    isNeedDisplayed = false;
  }

  // Print DC values.
  uint32_t currentMillis = millis();
  if (currentMillis - lastLcdUpdate >= LCD_INTERVAL)
  {
    lastLcdUpdate = currentMillis;
    uint8_t chargeLevel = batteryChargeLevel(battery);
    uint32_t current = calculateACS712Current(adcCurrent, currentZeroOffset);
    showParams(mode, main, stepDown, battery, current, chargeLevel);
  }
}

// Adds spaces. Padding the string to 16 characters.
void addSpaces()
{
  int len = strlen(displayBuffer);
  if (len > 16)
  {
    len = 16;
  }

  // Fill from current end to the 15th index.
  for (int i = len; i < 16; i++)
  {
    displayBuffer[i] = ' ';
  }

  displayBuffer[16] = '\0';
}

// Starts the process of displaying messages on the screen.
void printMessage(const char *text)
{
  strncpy(displayBuffer, text, sizeof(displayBuffer) - 1);
  displayBuffer[sizeof(displayBuffer) - 1] = '\0';
  if (strcmp(displayBuffer, previousDisplayBuffer) == 0)
  {
    return;
  }
  strcpy(previousDisplayBuffer, displayBuffer);
  isNeedDisplayed = true;
}

// Displays voltages and currents on the display.
void showParams(OperationMode mode, uint32_t main, uint32_t stepDown, uint32_t battery,
                uint32_t current, uint8_t chargeLevel)
{
  switch (mode)
  {
  case OperationMode::Startup:
    showVoltages(main, stepDown, battery);
    break;

  case OperationMode::StartupDelay:
    showVoltages(main, stepDown, battery);
    break;

  case OperationMode::Normal:
    showOutputParams(false, chargeLevel, current);
    break;

  case OperationMode::Battery:
    showOutputParams(true, chargeLevel, current);
    break;

  case OperationMode::LineAfterBattery:
    showOutputParams(true, chargeLevel, current);
    break;

  case OperationMode::HardwareError:
    showVoltages(main, stepDown, battery);
    break;
  }
}

// Split raw voltage values (in mV or scaled units).
VoltageDisplay parseVoltage(uint32_t rawValue, Precision type)
{
  VoltageDisplay value;

  // Calculate the whole number (e.g., 12850 -> 12).
  value.whole = rawValue / 1000;

  // Extract the first decimal digit (e.g., 12850 % 1000 = 850; 850 / 10 = 85).
  // Note: Use / 100 if you only need one digit (0-9).
  value.decimal = (rawValue % 1000) / static_cast<uint8_t>(type);

  return value;
}

// Displays voltages of main dc, output step down, battery.
void showVoltages(uint32_t mainV, uint32_t stepDownV, uint32_t batteryV)
{
  char line[17];

  // Предполагаем, что parseVoltage теперь принимает uint32_t (милливольты)
  VoltageDisplay mainLine = parseVoltage(mainV, Precision::Hundredths);
  VoltageDisplay stepDown = parseVoltage(stepDownV, Precision::Hundredths);
  VoltageDisplay battery = parseVoltage(batteryV, Precision::Tenths);

  // Формируем строку
  snprintf(line, sizeof(line), "%2d.%1d %2d.%1d %2d.%1d",
           mainLine.whole, mainLine.decimal,
           stepDown.whole, stepDown.decimal,
           battery.whole, battery.decimal);

  lcd.setCursor(0, 1);
  lcd.print(line);
}

// Displays the output parameters on the LCD second line.
void showOutputParams(bool isBatteryMode, uint8_t chargeLevel,
                      uint32_t current)
{
  char line[17]; // Buffer for LCD (16 chars + null terminator)
  VoltageDisplay loadCurrent = parseVoltage(current, Precision::Hundredths);

  if (isBatteryMode)
  {
    // Counting line off time.
    bool showSeparator = lineOffClock();

    // Format for Battery Mode: "85%  1.78 02:15 "
    snprintf(line, sizeof(line), "%d%% %2d.%02d %02u%c%02u ",
         chargeLevel,
         loadCurrent.whole, loadCurrent.decimal,
         linePowerOffClock.hours,
         showSeparator ? ':' : ' ',
         linePowerOffClock.minutes);
  }
  else
  {
    lineOffClockReset();

    // Format for Normal Mode: "100%  1.78A      "
    snprintf(line, sizeof(line), "%d%% %2d.%02dA      ",
             chargeLevel,
             loadCurrent.whole, loadCurrent.decimal);
  }

  // Write the formatted string to the second line of the LCD
  lcd.setCursor(0, 1);
  lcd.print(line);
}

// Calculates the battery charge level in percent.
uint8_t batteryChargeLevel(uint32_t voltage_mv)
{
  uint32_t voltage = constrain(voltage_mv,
                               BATTERY_MIN_MV, BATTERY_MAX_MV);

  uint32_t range = BATTERY_MAX_MV - BATTERY_MIN_MV;
  if (range == 0 || voltage < BATTERY_MIN_MV)
  {
    return 0;
  }

  uint32_t relativeVoltage = voltage - BATTERY_MIN_MV;

  return (relativeVoltage * 100UL) / range;
}

// Calculates the load current in milliamperes (mA).
uint32_t calculateACS712Current(uint32_t adcValue, uint32_t zeroOffset)
{
  // Get current in mA (e.g., 1780 for 1.78А.)
  int32_t sensotVoltage = (adcValue - zeroOffset) - ACS_OFFSET;
  if (sensotVoltage < 0)
  {
    sensotVoltage = 0;
  }

  return abs((sensotVoltage) * 1000L / ACS_SENSITIVITY);
}

// Calculating the line outage time.
bool lineOffClock()
{
  uint32_t now = millis();
  bool showSeparator = (now / 500 % 2 == 0);
  if(lineOffStartTime == 0)
  {
    lineOffStartTime = now; 
  }

  if (now - lastClockTick >= 1000)
  {
    lastClockTick = now;

    uint32_t elapsedMs = now - lineOffStartTime;
    uint32_t totalSeconds = elapsedMs / 1000;
    linePowerOffClock.seconds = totalSeconds % 60;
    linePowerOffClock.minutes = (totalSeconds / 60) % 60;
    linePowerOffClock.hours   = (totalSeconds / 3600);
  }

  return showSeparator;
}

// Resets the line outage clock.
void lineOffClockReset()
{
  lineOffStartTime = 0;
  lastClockTick = 0;
  linePowerOffClock.seconds = 0;
  linePowerOffClock.minutes = 0;
  linePowerOffClock.hours = 0;
}