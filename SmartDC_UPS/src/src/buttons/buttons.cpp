#include "buttons.h"

uint32_t btnOffLoadTimer;
bool btnOffLoadLock;
bool btnSlaveLoadLock;
uint32_t btnSlaveLoadTimer;

void buttonsInit()
{
  // Initialize digital pins as inputs.
  // TTP223 sensors output a HIGH signal when touched.
  pinMode(BTN_MAIN_LOAD_PIN, INPUT);
  pinMode(BTN_SLAVE_LOAD_PIN, INPUT);
}


void buttonLongPressCheck(uint8_t pin, uint32_t *btnTimer, bool *targetFlag, bool *isLocked)
{
  uint32_t currentTime = millis() / 1000;

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

void readButtons(bool* mainLoadStatus, bool* slaveLoadStatus)
{
  // Off/On main load.
  buttonLongPressCheck(BTN_MAIN_LOAD_PIN, &btnOffLoadTimer,
                       mainLoadStatus, &btnOffLoadLock);

  // Off/On slave load.
  buttonLongPressCheck(BTN_SLAVE_LOAD_PIN, &btnSlaveLoadTimer,
                       slaveLoadStatus, &btnSlaveLoadLock);
}
