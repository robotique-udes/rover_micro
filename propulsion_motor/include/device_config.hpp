#ifndef DEVICE_CONFIG_HPP
#define DEVICE_CONFIG_HPP

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "rover_can2/constant.hpp"

constexpr RoverCan2::Constant::eDeviceId DEVICE_ID = RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR;

constexpr float ALIM_VOLTAGE = 24.0F;
constexpr float MAX_VOLTAGE = 12.0F;

#endif  // DEVICE_CONFIG_HPP
