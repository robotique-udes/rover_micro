#ifndef CONFIG_LOCAL_HPP
#define CONFIG_LOCAL_HPP

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "rover_can2/constant.hpp"

constexpr RoverCan2::Constant::eDeviceId DEVICE_ID = RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR;

constexpr float ALIM_VOLTAGE = 24.0F;
constexpr float MAX_VOLTAGE = 12.0F; // Not higher than 18V for BackEMF problems
 
#endif  // __CONFIG_LOCAL_HPP__
