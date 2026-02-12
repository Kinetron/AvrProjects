#include "control_logic.h"

SystemParameters params =
    {
        OperationMode::Startup,
        false
    };

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
  
  blinkBuiltinLedInit();
  
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
  buttonsInit();
  fanInit();

  // Full charge current.
  digitalWrite(TURN_ON_FULL_CHARGE_PIN, HIGH);
  pinMode(TURN_ON_FULL_CHARGE_PIN, OUTPUT);

  initLCD();
}

void logic_tick()
{
  readAdc();
  checkLine();
  switchModes();
  protectOvervoltage();

  displayService(
    params.mode, 
    voltages[MAIN_VOLTAGE_INDEX], 
    voltages[STEP_DOWN_VOLTAGE_INDEX],
    voltages[BATTERY_VOLTAGE_INDEX], 
    voltages[LOAD_CURRENT_INDEX],
    params.ACS712zeroOffset
  );

  monitorLineRecovery();
  blinkBuiltinLedProcess();
  readButtons(&params.OffMainLoad, &params.OffSlaveLoadFromBtn);
  fanControl(voltages[TEMPERATURE_SENSOR_VOLTAGE_INDEX]);
  chargelogic();
  onSecondLoadDelegate();
}

void switchModes()
{
  switch (params.mode)
  {
  case OperationMode::Startup:
    startupStep();
    break;

  case OperationMode::StartupDelay:
    offBlinkBuiltinLed();
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
    blinkBuiltinLed(true, BLINK_INTERVAL_03S);
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
  blinkBuiltinLed(true, BLINK_INTERVAL_1S);

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
    if (!params.OffSlaveLoadFromBtn && !params.OffSlaveLoad)
    {
      params.onSecondLoad = true; 
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
    params.onSecondLoad = !params.OffSlaveLoadFromBtn && !params.OffSlaveLoad;
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
    params.connectingToMainSource = true;
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
  // Measurement delay before connecting to the main power supply.
  bool notCheckMainDc = false;
  if(params.connectingToMainSource)
  {
    notCheckMainDc = lockAnalysisVoltageMainSourceTimer();
    if(!notCheckMainDc)
    {
      resetAnalysisVoltageMainSourceTimer();
      params.connectingToMainSource = false;
    }
  }
  
  if (!notCheckMainDc && params.mode == OperationMode::Normal && !checkMainDcVoltage())
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
  if (params.lineOnTimer == 0 ||
      params.lineOnTimer > getSeconds())
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
  if (params.attemptsAfterlineOn == 0 ||
      params.attemptsAfterlineOn > getSeconds())
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
  blinkBuiltinLed(true, BLINK_INTERVAL_1S);

  snprintf(buffer, sizeof(buffer), "L OK.Wait%d/%d",
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
    offBlinkBuiltinLed();
    params.mode = OperationMode::Battery;
    return;
  }

  // Delay.
  if (params.lineOn && lineOnTimer())
  {
    resetlineOnTimer();
    resetAttemptsAfterlineOnTimer();
    offBlinkBuiltinLed();
    params.mode = OperationMode::Normal;
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
  if (params.onFullChargeTimer == 0 ||
      params.onFullChargeTimer > time)
  {
    params.onFullChargeTimer = time;
  }

  if (time - params.onFullChargeTimer > FULL_CHARGE_DELAY)
  {
    return true;
  }

  return false;
}

// Resets the full charge timer.
void resetFullChargeTimer()
{
  params.onFullChargeTimer = 0;
}

// Checks if the ACS712 calibration interval has elapsed.
bool ACS712calibrateTimer()
{
  uint32_t time = getSeconds();
  if (params.ACS712calibrateTimer == 0 ||
      params.ACS712calibrateTimer > time)
  {
    params.ACS712calibrateTimer = time;
  }

  if (time - params.ACS712calibrateTimer > ACS712_CALIBRATE_TIMER_DELAY)
  {
    return true;
  }

  return false;
}

// Resets the timer for ACS712 sensor calibration.
void resetACS712calibrateTimer()
{
  params.ACS712calibrateTimer = 0;
}

// Calibrates the ACS712 current sensor.
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
    resetOffSecondLoadTimer();
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

// Resets the second load outage interval timer.
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

// Resets the second load outage timer.
void resetOffSecondLoadTimer()
{
  params.periodOffSecondLoadTimer = 0;
}

// When connected to the load, it blocks the measurement of voltage on the main power supply unit.
bool lockAnalysisVoltageMainSourceTimer()
{
  uint32_t time = getSeconds();
  if (params.lockAnalysisVoltageMainSourceTimer == 0 ||
      params.lockAnalysisVoltageMainSourceTimer > time)
  {
    params.lockAnalysisVoltageMainSourceTimer = time;
  }

  if (time - params.lockAnalysisVoltageMainSourceTimer > 
      LOCK_ANALYSIS_VOLTAGE_MAIN_SOURCE_DELAY)
  {
    return false;
  }

  return true;    
}

void resetAnalysisVoltageMainSourceTimer()
{
  params.lockAnalysisVoltageMainSourceTimer = 0;
} 

// Load connection delay.
bool delayOnSecondLoadTimer()
{
  uint32_t time = getSeconds();
  if (params.delayOnSecondLoadTimer == 0 ||
      params.delayOnSecondLoadTimer > time)
  {
    params.delayOnSecondLoadTimer = time;
  }

  if (time - params.delayOnSecondLoadTimer > 
      SECOND_LOAD_ON_DELAY)
  {
    return false;
  }

  return true;
}

void resetDelayOnSecondLoadTimer()
{
  params.delayOnSecondLoadTimer = 0;
}

void onSecondLoadDelegate()
{
  if(!params.onSecondLoad)
  {
    digitalWrite(SLAVE_LOAD_ON_PIN, LOW);
    return;
  }  

  // Delay on.
  if(!digitalRead(SLAVE_LOAD_ON_PIN) && !delayOnSecondLoadTimer())
  {
     digitalWrite(SLAVE_LOAD_ON_PIN, HIGH);
     resetDelayOnSecondLoadTimer();
  }
}
