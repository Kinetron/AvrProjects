#include "control_logic.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

SystemParameters params =
    {
        OperationMode::Startup,
        false};

char displayBuffer[17];
char previousDisplayBuffer[17];

uint32_t lastMeasureTime = 0;
uint32_t voltages[NUM_CHANNELS]; // Values in millivolts

bool isAdcConverting = false;
uint8_t currentAdcChannel = 0;
uint32_t channelSum = 0;
uint64_t adcWithCoeff;
uint8_t sampleCount = 0;

// The time interval from line voltage.
volatile uint32_t line_last_pulse_time = 0;
volatile uint32_t line_pulse_time = 0;

volatile bool pulseDetected = false;

void logic_init()
{
  pinMode(LINE_SENSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LINE_SENSE_PIN), line_pulse_isr, RISING);

  // Set ADC reference to AVCC (5V). REFS0 = 1, REFS1 = 0
  ADMUX = (1 << REFS0);

  // Enable ADC and set Prescaler to 128 (16MHz/128 = 125kHz ADC clock) for ATmega328P.
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Relays.
  digitalWrite(RELAY_PSU_PIN, HIGH);
  digitalWrite(RELAY_BAT_PIN, HIGH);
  pinMode(RELAY_PSU_PIN, OUTPUT);
  pinMode(RELAY_BAT_PIN, OUTPUT);

  // Slave load on/off.
  digitalWrite(SLAVE_LOAD_ON_PIN, LOW);
  pinMode(SLAVE_LOAD_ON_PIN, OUTPUT);

  // On/Off main load mosfet.
  digitalWrite(MAIN_LOAD_ON_PIN, LOW);
  pinMode(MAIN_LOAD_ON_PIN, OUTPUT);
  params.lastPowerState = false;

  // Step down source.
  digitalWrite(RELAY_STEPDOWN_SWITCH_SOURCE_PIN, HIGH);
  pinMode(RELAY_STEPDOWN_SWITCH_SOURCE_PIN, OUTPUT);

  // Initialize digital pins as inputs.
  // TTP223 sensors output a HIGH signal when touched.
  pinMode(BTN_MAIN_LOAD_PIN, INPUT);
  pinMode(BTN_SLAVE_LOAD_PIN, INPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  digitalWrite(FAN_PWM_PIN, HIGH); // Off fan.

  // Full charge current.
  digitalWrite(TURN_ON_FULL_CHARGE_PIN, HIGH);
  pinMode(TURN_ON_FULL_CHARGE_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
}

void logic_tick()
{
  readAdc();
  checkLine();
  switchModes();
  protectOvervoltage();
  displayService();
  monitorLineRecovery();
  blinkBuiltinLed();
  readButtons();
  fanControl();
  chargelogic();
}

void switchModes()
{
  switch (params.mode)
  {
  case OperationMode::Startup:
    startupStep();
    break;

  case OperationMode::StartupDelay:
    params.blinkBuiltinLedOn = false;
    workAfterload();
    break;

  case OperationMode::Normal:
    loadOn(LoadSource::MainPSU);
    autoOffSecondLoad();
    printMessage("Normal. AC ok.");
    break;

  case OperationMode::Battery:
    loadOn(LoadSource::Battery);
    autoOffSecondLoad();
    printMessage("Battery.");
    break;

  case OperationMode::LineAfterBattery:
    lineAfterBatteryStep();
    break;

  case OperationMode::HardwareError:
    params.blinkIntervalBuiltin = BLINK_INTERVAL_03S;
    params.blinkBuiltinLedOn = true;
    break;
  }
}

// Returns the number of seconds since the start.
uint32_t getSeconds()
{
  return millis() / 1000;
}

// Has there been a delay after turning on the device?
bool initDelayPassed()
{
  params.waitingTimePassed = getSeconds();
  if (params.waitingTimePassed >= STARTUP_DELAY)
  {
    params.waitingTimePassed = 0;
    return true;
  }

  return false;
}

void startupStep()
{
  char buffer[17];
  params.blinkIntervalBuiltin = BLINK_INTERVAL_1S;
  params.blinkBuiltinLedOn = true;

  calibrateACS712();

  if (params.lineOn)
  {
    snprintf(buffer, sizeof(buffer), "AC ok.Wait%d/%ds", params.waitingTimePassed,
             STARTUP_DELAY);
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "Wait%d/%ds", params.waitingTimePassed,
             STARTUP_DELAY);
  }

  printMessage(buffer);

  if (initDelayPassed())
  {
    params.mode = OperationMode::StartupDelay;
  }
}

// Switch power sources.
void switchPowerSources(LoadSource sourceType)
{
  // Need to turn off the load.
  if (params.lastPowerState && sourceType == LoadSource::None)
  {
    // Off main load mosfet.
    digitalWrite(MAIN_LOAD_ON_PIN, LOW);
    // Off slave load MOSFET.
    digitalWrite(SLAVE_LOAD_ON_PIN, LOW);
    delay(SHUTDOWN_TIME_MAIN_LOAD_MOSFET);
    params.lastPowerState = false;
  }

  // Active low level.
  switch (sourceType)
  {
  case LoadSource::MainPSU:
    digitalWrite(RELAY_BAT_PIN, HIGH);
    digitalWrite(RELAY_PSU_PIN, LOW);
    break;

  // Connect to the battery.
  case LoadSource::Battery:
    digitalWrite(RELAY_PSU_PIN, HIGH);
    digitalWrite(RELAY_BAT_PIN, LOW);
    break;

  // Disconnect everything.
  case LoadSource::None:
  default:
    digitalWrite(RELAY_PSU_PIN, HIGH);
    digitalWrite(RELAY_BAT_PIN, HIGH);
    break;
  }

  // Need to turn on the load.
  if (!params.lastPowerState && sourceType != LoadSource::None)
  {
    delay(RELAY_SWITCH_TIME);
    // On main load mosfet.
    if (!params.OffMainLoad)
    {
      digitalWrite(MAIN_LOAD_ON_PIN, HIGH);
    }

    // On slave load MOSFET.
    if (!params.OffSlaveLoad)
    {
      digitalWrite(SLAVE_LOAD_ON_PIN, HIGH);
    }
    params.lastPowerState = true;
  }
}

// Connect the load to an energy source.
void loadOn(LoadSource sourceType)
{
  // Plus of power supply.
  if (params.lastPowerState)
  {
    digitalWrite(MAIN_LOAD_ON_PIN, !params.OffMainLoad);
    digitalWrite(SLAVE_LOAD_ON_PIN, !params.OffSlaveLoad);
  }

  switchPowerSources(sourceType);

  // Switch step down input.
  switch (sourceType)
  {
  case LoadSource::MainPSU:
    // Active low level.
    // Connect the input to the boost converter.
    digitalWrite(RELAY_STEPDOWN_SWITCH_SOURCE_PIN, LOW);
    break;

  case LoadSource::Battery:
    // Connect to the battery.
    digitalWrite(RELAY_STEPDOWN_SWITCH_SOURCE_PIN, HIGH);
    break;

  case LoadSource::None:
  default:
    // Connect to the battery.
    digitalWrite(RELAY_STEPDOWN_SWITCH_SOURCE_PIN, HIGH);
    break;
  }
}

void workAfterload()
{
  // Сheck the voltage of the main power supply.
  if (params.lineOn && !checkMainDcVoltage())
  {
    printMessage("Main dc error.");
    return;
  }

  if (params.lineOn)
  {
    params.mode = OperationMode::Normal;
    return;
  }

  //  Step down is not broken and the voltage is normal.
  if (!checkStepDownVoltage())
  {
    printMessage("Step down error.");
    return;
  }

  // The battery voltage increased, but the battery was not charging.
  if (voltages[BATTERY_VOLTAGE_INDEX] < BATTERY_LOWER_THRESHOLD)
  {
    printMessage("Battery low.");
    return;
  }

  params.mode = OperationMode::Battery;
  return;
}

bool checkMainDcVoltage()
{
  return (voltages[MAIN_VOLTAGE_INDEX] < MAIN_UPPER_THRESHOLD &&
          voltages[MAIN_VOLTAGE_INDEX] > MAIN_DOWN_LOWER_THRESHOLD);
}

// Checks the output voltage step down.
bool checkStepDownVoltage()
{
  return voltages[STEP_DOWN_VOLTAGE_INDEX] < STEP_DOWN_UPPER_THRESHOLD &&
         voltages[STEP_DOWN_VOLTAGE_INDEX] > STEP_DOWN_LOWER_THRESHOLD;
}

void protectOvervoltage()
{
  if (params.mode == OperationMode::Normal && !checkMainDcVoltage())
  {
    loadOn(LoadSource::None);
    params.mode = OperationMode::HardwareError;
    printMessage("Main dc error.");
  }

  if (params.mode == OperationMode::Battery && !checkStepDownVoltage())
  {
    loadOn(LoadSource::None);
    params.mode = OperationMode::HardwareError;
    printMessage("Step down error.");
  }
}

// Delay after AC line recovery.
bool lineOnTimer()
{
  if (params.lineOnTimer == 0)
  {
    params.lineOnTimer = getSeconds();
    params.waitingTimePassed = 0;
  }

  uint32_t diff = getSeconds() - params.lineOnTimer;
  params.waitingTimePassed = diff;
  if (diff >= LINE_ON_DELAY)
  {
    params.waitingTimePassed = 0;
    return true;
  }

  return false;
}

// Reset line on timer.
void resetlineOnTimer()
{
  params.lineOnTimer = 0;
}

// Timer for waiting for stable line voltage.
bool attemptsAfterlineOnTimer()
{
  if (params.attemptsAfterlineOn == 0)
  {
    params.attemptsAfterlineOn = getSeconds();
  }

  if (getSeconds() - params.attemptsAfterlineOn >= LINE_ON_ATTEMPTS)
  {
    return true;
  }

  return false;
}

// Reset timer for waiting for stable line voltage.
void resetAttemptsAfterlineOnTimer()
{
  params.attemptsAfterlineOn = 0;
}

// The waiting step after the voltage appears in the line.
void lineAfterBatteryStep()
{
  char buffer[17];
  params.blinkIntervalBuiltin = BLINK_INTERVAL_1S;
  params.blinkBuiltinLedOn = true;

  snprintf(buffer, sizeof(buffer), "Line OK.Wait%d/%d",
           params.waitingTimePassed, LINE_ON_DELAY);
  printMessage(buffer);

  // Sparks and interruptions in the line.
  if (!params.lineOn)
  {
    resetlineOnTimer();
  }

  // We did not receive a stable voltage for this time.
  if (attemptsAfterlineOnTimer())
  {
    resetAttemptsAfterlineOnTimer();
    resetlineOnTimer();
    params.blinkBuiltinLedOn = false;
    params.mode = OperationMode::Battery;
    return;
  }

  // Delay.
  if (params.lineOn && lineOnTimer())
  {
    resetlineOnTimer();
    resetAttemptsAfterlineOnTimer();
    params.blinkBuiltinLedOn = false;
    params.mode = OperationMode::Normal;
  }
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
  params.isNeedDisplayed = true;
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

// Displays messages and voltage on the screen.
void displayService()
{
  // Print text if change.
  if (params.isNeedDisplayed)
  {
    addSpaces();
    lcd.setCursor(0, 0);
    lcd.print(displayBuffer);
    params.isNeedDisplayed = false;
  }

  // Print DC values.
  uint32_t currentMillis = millis();
  if (currentMillis - params.lastLcdUpdate >= LCD_INTERVAL)
  {
    params.lastLcdUpdate = currentMillis;
    showParams();
  }
}

void readAdc()
{
  // New read.
  if (!isAdcConverting)
  {
    ADMUX = (ADMUX & 0xF0) | (ADC_PINS[currentAdcChannel] & 0x07); // Switch pin.
    ADCSRA |= (1 << ADSC);                                         // Run ADC.
    isAdcConverting = true;
    return;
  }

  // ADC is busy.
  if (ADCSRA & (1 << ADSC))
  {
    return;
  }

  channelSum += ADC;
  sampleCount++;
  isAdcConverting = false;

  if (sampleCount >= SAMPLES_COUNT)
  {
    checkLine(); // For fast reaction.
    adcWithCoeff = (uint64_t)channelSum * ADC_COEFFS[currentAdcChannel];
    voltages[currentAdcChannel] = (uint32_t)(adcWithCoeff >> SAMPLES_SHIFT);
    channelSum = 0;
    sampleCount = 0;
    currentAdcChannel++;
    if (currentAdcChannel >= NUM_CHANNELS)
    {
      currentAdcChannel = 0;
    }
  }
}

// Rising pulse line voltage. Every 10ms.
void line_pulse_isr()
{
  line_last_pulse_time = millis();
}

// Сheck line voltage.
void checkLine()
{
  noInterrupts();
  line_pulse_time = line_last_pulse_time;
  interrupts();

  if ((millis() - line_pulse_time) < LINE_TIMEOUT_MS)
  {
    params.lineOn = true;
    return;
  }

  // Emergency switch to battery.
  if (params.mode == OperationMode::Normal && checkStepDownVoltage())
  {
    loadOn(LoadSource::Battery);
    params.mode = OperationMode::Battery;
  }

  params.lineOn = false;
}

// Monitors the moment when the line voltage is restored.
void monitorLineRecovery()
{
  if (params.mode != OperationMode::Battery)
  {
    return;
  }

  if (params.lineOn)
  {
    params.mode = OperationMode::LineAfterBattery;
  }
}

void buttonLongPressCheck(uint8_t pin, uint32_t *btnTimer, bool *targetFlag, bool *isLocked)
{
  uint32_t currentTime = getSeconds();

  // Check if the sensor is currently touched.
  if (digitalRead(pin))
  {
    // If the timer is not running, start it now. Or if it work > 49days.
    if (*btnTimer == 0 || currentTime < *btnTimer)
    {
      *btnTimer = currentTime;
    }

    // Check if the holding time has exceeded our threshold
    // Also check if we haven't toggled the flag yet during this press (isLocked)
    if (!(*isLocked) && (currentTime - *btnTimer >= BTN_HOLD_TIME))
    {
      *targetFlag = !(*targetFlag); // Switch 0 to 1 or 1 to 0
      *isLocked = true;             // Lock further toggles until finger is removed
    }
  }
  else
  {
    // Reset everything for the next touch.
    *btnTimer = 0;
    *isLocked = false;
  }
}

void readButtons()
{
  // Off/On main load.
  buttonLongPressCheck(BTN_MAIN_LOAD_PIN, &params.btnOffLoadTimer,
                       &params.OffMainLoad, &params.btnOffLoadLock);

  // Off/On slave load.
  buttonLongPressCheck(BTN_SLAVE_LOAD_PIN, &params.btnSlaveLoadTimer,
                       &params.OffSlaveLoad, &params.btnSlaveLoadLock);
}

// Blink built-in led on the arduino nano board.
void blinkBuiltinLed()
{
  if (!params.blinkBuiltinLedOn)
  {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  uint32_t currentTime = millis();

  if (currentTime - params.lastBlinkTimeBuiltin >= params.blinkIntervalBuiltin)
  {
    params.lastBlinkTimeBuiltin = currentTime;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

// Displays voltages and currents on the display.
void showParams()
{
  switch (params.mode)
  {
  case OperationMode::Startup:
    showVoltages();
    break;

  case OperationMode::StartupDelay:
    showVoltages();
    break;

  case OperationMode::Normal:
    showOutputParams(LoadSource::MainPSU);
    break;

  case OperationMode::Battery:
    showOutputParams(LoadSource::Battery);
    break;

  case OperationMode::LineAfterBattery:
    showOutputParams(LoadSource::Battery);
    break;

  case OperationMode::HardwareError:
    showVoltages();
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
void showVoltages()
{
  char line[17]; // Buffer for LCD (16 chars + null terminator)

  // Сhannel 0 (A0). Main DC(e.g., 12.81).
  VoltageDisplay mainLine = parseVoltage(voltages[MAIN_VOLTAGE_INDEX], Precision::Hundredths);

  // Channel 1 (A1). Output step down(e.g., 12.83).
  VoltageDisplay stepDown = parseVoltage(voltages[STEP_DOWN_VOLTAGE_INDEX], Precision::Hundredths);

  // Channel 2 (A2). Battery(e.g., 28.7)
  VoltageDisplay battery = parseVoltage(voltages[BATTERY_VOLTAGE_INDEX], Precision::Tenths);

  snprintf(line, sizeof(line), "%2d.%1d %2d.%1d %2d.%1d", mainLine.whole, mainLine.decimal,
           stepDown.whole, stepDown.decimal, battery.whole, battery.decimal);

  lcd.setCursor(0, 1);
  lcd.print(line);
}

// Shows the output voltage and current on the display.
void showOutputParams(LoadSource sourceType)
{
  char line[17];

  uint8_t chargeLevel = batteryChargeLevel();

  // Get current in mA (e.g., 1780 for 1.78А.)
  int32_t sensotVoltage = (voltages[LOAD_CURRENT_INDEX] - params.ACS712zeroOffset) - ACS_OFFSET;
  if (sensotVoltage < 0)
  {
    sensotVoltage = 0;
  }

  uint32_t loadCurrentRaw = abs((sensotVoltage) * 1000L / ACS_SENSITIVITY);
  VoltageDisplay loadCurrent = parseVoltage(loadCurrentRaw, Precision::Hundredths);

  if (params.mode == OperationMode::Battery)
  {
    // Counting line off time.
    lineOffClock();
    snprintf(line, sizeof(line), "%d%% %2d.%02d %02u:%02u ",
             chargeLevel,
             loadCurrent.whole, loadCurrent.decimal,
             params.lineOffClock.hours,
             params.lineOffClock.minutes);
  }
  else
  {
    lineOffClockReset();
    snprintf(line, sizeof(line), "%d%% %2d.%02dA      ",
             chargeLevel,
             loadCurrent.whole, loadCurrent.decimal);
  }

  lcd.setCursor(0, 1);
  lcd.print(line);
}

uint8_t batteryChargeLevel()
{
  uint32_t voltage = constrain(voltages[BATTERY_VOLTAGE_INDEX],
                               BATTERY_MIN_MV, BATTERY_MAX_MV);

  uint32_t range = BATTERY_MAX_MV - BATTERY_MIN_MV;
  if (voltage < BATTERY_MIN_MV)
  {
    return 0;
  }

  uint32_t relativeVoltage = voltage - BATTERY_MIN_MV;

  return (relativeVoltage * 100UL) / range;
}

// Controls the fan speed.
void fanControl()
{
  uint32_t now = millis();

  // Run fan control logic every x seconds.
  if (now - params.fanLastLogicTime < FAN_REACTION_TIME)
  {
    return;
  }

  params.fanLastLogicTime = now;

  // 2414mv at 24 degrees
  uint32_t voltage = voltages[TEMPERATURE_SENSOR_VOLTAGE_INDEX];
  uint8_t targetSpeed = 0;

  // Hysteresis logic
  if (voltage > TEMP_ON)
  {
    // Map the input value to PWM range.
    int32_t mapped = map(voltage, TEMP_ON, TEMP_MAX, SPEED_MIN, SPEED_MAX);
    targetSpeed = (uint8_t)constrain(mapped, SPEED_MIN, SPEED_MAX);
  }
  else if (voltage < TEMP_OFF)
  {
    // Turn off if below the lower threshold.
    targetSpeed = 0;
  }
  else
  {
    // Keep previous state if value is within the "dead zone" (hysteresis).
    targetSpeed = params.fanCurrentSpeed;
  }

  // Check if we need to start a kickstart burst
  if (targetSpeed > 0 && params.fanCurrentSpeed == 0)
  {
    params.fanKickstopTime = millis() + KICK_MS;
  }

  params.fanCurrentSpeed = targetSpeed;

  if (millis() < params.fanKickstopTime)
  {
    analogWrite(FAN_PWM_PIN, 0); // Max speed.
  }
  else
  {
    analogWrite(FAN_PWM_PIN, SPEED_MAX - params.fanCurrentSpeed);
  }
}

// Controls the battery charge current.
void chargelogic()
{
  if (!params.lineOn)
  {
    resetFullChargeTimer();
    // Off maximum charging current.
    digitalWrite(TURN_ON_FULL_CHARGE_PIN, HIGH);
    params.fullChargeOn = false;
    return;
  }

  // Delay on.  23100
  if (!params.fullChargeOn &&
     voltages[BATTERY_VOLTAGE_INDEX] >=  MIN_THRESHOLD_BATTERY_FULL_CHARGE_ON
     && fullChargeTimer())
  {
    digitalWrite(TURN_ON_FULL_CHARGE_PIN, LOW);
    resetFullChargeTimer();
    params.fullChargeOn = true;
  }
}

bool fullChargeTimer()
{
  uint32_t time = getSeconds();
  if (params.onFullChargeTimer == 0 || params.onFullChargeTimer > time)
  {
    params.onFullChargeTimer = time;
  }

  if (time - params.onFullChargeTimer > FULL_CHARGE_DELAY)
  {
    return true;
  }

  return false;
}

void resetFullChargeTimer()
{
  params.onFullChargeTimer = 0;
}

bool ACS712calibrateTimer()
{
  uint32_t time = getSeconds();
  if (params.ACS712calibrateTimer == 0 || params.ACS712calibrateTimer > time)
  {
    params.ACS712calibrateTimer = time;
  }

  if (time - params.ACS712calibrateTimer > ACS712_CALIBRATE_TIMER_DELAY)
  {
    return true;
  }

  return false;
}

void resetACS712calibrateTimer()
{
  params.ACS712calibrateTimer = 0;
}

void calibrateACS712()
{
  if (params.isCalibrateACS712)
  {
    return;
  }

  if (!ACS712calibrateTimer())
  {
    return;
  }

  params.ACS712zeroOffset = voltages[LOAD_CURRENT_INDEX];
  params.isCalibrateACS712 = true;
}

// Turns off the secondary load after a certain time after the line drops.
void autoOffSecondLoad()
{
  if (params.lastOffSecondLoadStatus &&
      params.mode == OperationMode::Normal)
  {
    params.OffSlaveLoad = false;
    params.lastOffSecondLoadStatus = false;
  }

  if (params.mode == OperationMode::Battery &&
      !params.lastOffSecondLoadStatus &&
      offSecondLoadTimer())
  {
    params.OffSlaveLoad = true;
    params.lastOffSecondLoadStatus = true;
    resetOffSecondLoadTimer();
  }
}

bool offSecondLoadTimer()
{
  uint32_t time = getSeconds();
  if (params.periodOffSecondLoadTimer == 0 ||
      params.periodOffSecondLoadTimer > time)
  {
    params.periodOffSecondLoadTimer = time;
  }

  if (time - params.periodOffSecondLoadTimer > OFF_TIME_SECOND_LOAD)
  {
    return true;
  }

  return false;
}

void resetOffSecondLoadTimer()
{
  params.periodOffSecondLoadTimer = 0;
}

void lineOffClock()
{
  uint32_t time = millis();
  if (time - params.lastClockTick >= 1000)
  {
    params.lastClockTick = time;
    params.lineOffClock.seconds++;

    if (params.lineOffClock.seconds >= 60)
    {
      params.lineOffClock.seconds = 0;
      params.lineOffClock.minutes++;
      if (params.lineOffClock.minutes >= 60)
      {
        params.lineOffClock.minutes = 0;
        params.lineOffClock.hours++;
      }
    }
  }
}

void lineOffClockReset()
{
  params.lineOffClock.seconds = 0;
  params.lineOffClock.minutes = 0;
  params.lineOffClock.hours = 0;
}