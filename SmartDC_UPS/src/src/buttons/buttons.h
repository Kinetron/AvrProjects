#include <Arduino.h>

#define BTN_MAIN_LOAD_PIN 6
#define BTN_SLAVE_LOAD_PIN 5
#define BTN_HOLD_TIME 3 // 3 sec

void buttonsInit();

void buttonLongPressCheck(uint8_t pin, uint32_t *btnTimer, bool *targetFlag, bool *isLocked);

void readButtons(bool* mainLoadStatus, bool* slaveLoadStatus);