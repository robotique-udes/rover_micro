#ifndef __ROVER_ROS_SERIAL_HPP__
#define __ROVER_ROS_SERIAL_HPP__

#if defined(ESP32)

#include "Arduino.h"
#include "helpers/timer.h"
#include "helpers/macros.h"
#include "helpers/log.h"

#endif // defined(ESP32)

#include "rover_ros_serial/base_objects.hpp"
#include "rover_ros_serial/msg.hpp"

#if defined(ESP32)

#include "logger.hpp"
#include "heartbeat.hpp"
#include "node.hpp"

#endif

#endif // __ROVER_ROS_SERIAL_HPP__
