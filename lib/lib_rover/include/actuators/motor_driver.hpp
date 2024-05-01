#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

// #if !defined(ESP32)
// #error CPU is not supported
// #else

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

enum eDriverType
{
  _talonSRX,
  _empty
};

class MotorDriver
{
public:
  MotorDriver(){};
  virtual ~MotorDriver(){};
  virtual void init(float arg1, float arg2) = 0;
  virtual void setSpd(float spd) = 0;
  // virtual void stopMot(void) = 0;
  virtual void enableMot(void) = 0;
  virtual void disableMot(void) = 0;
  virtual void resetMot(void) = 0;
  virtual bool isMoving(void) = 0;
};

// #endif // !defined(ESP32)
#endif // __MOTOR_DRIVER_HPP__
