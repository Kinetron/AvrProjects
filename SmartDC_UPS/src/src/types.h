#ifndef TYPES_H
#define TYPES_H 

// Structure for convenient time storage
struct TimeValues
{
  uint8_t hours; // Clock
  uint8_t minutes; // Minutes
  uint8_t seconds; // Seconds
};

// Operating modes of the control logic.
enum class OperationMode : uint8_t
{
  Startup,          // Initial system boot.
  StartupDelay,     // Waiting for stabilization (e.g., relay protection).
  Normal,           // Power from AC-DC unit.
  Battery,          // Power from battery.
  LineAfterBattery, //
  HardwareError     // Step down/main overvoltage.
};

#endif