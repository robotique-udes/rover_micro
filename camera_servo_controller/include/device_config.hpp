#ifndef CONFIG_LOCAL_HPP
#define CONFIG_LOCAL_HPP

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "rover_can2/constant.hpp"
#include "rover_lib2/actuators/servo.hpp"

enum class eServoType
{
    PAN,
    TILT,
    ZOOM
};

// ===============================================================================================================================
// Device specific
// ===============================================================================================================================
constexpr RoverCan2::Constant::eDeviceId DEVICE_ID = RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA;
#define PCB_REVISION_REV_0

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions after this necessary between uploads
// ===============================================================================================================================

#if defined(PCB_REVISION_REV_0)
constexpr gpio_num_t PIN_SERVO_1 = GPIO_NUM_15;
constexpr gpio_num_t PIN_SERVO_2 = GPIO_NUM_16;
constexpr gpio_num_t PIN_LED_4 = GPIO_NUM_1;  // Marked as led 4 on pcb schematics
constexpr gpio_num_t PIN_LED_5 = GPIO_NUM_2;  // Marked as led 3 on pcb schematics

constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_12;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_13;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_14;

template<eServoType servoType_>
constexpr Actuators::ServoT::sTimingConfig GET_SERVO_TIMING_CONFIG(void)
{
    if constexpr (DEVICE_ID == RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)
    {
        if constexpr (servoType_ == eServoType::PAN)
        {
            return Actuators::ServoT::sTimingConfig{
                .frequency = 50.0F,
                .minMs = 2450.0F,
                .maxMs = 545.0F,
                .minPosition = 0.0F,
                .maxPosition = static_cast<float>(M_PI * 2.0),
                .maxSpeed = 0.89F,
            };
        }
        else if constexpr (servoType_ == eServoType::TILT || servoType_ == eServoType::ZOOM)
        {
            static_assert(false, "eServoType::TILT and eServoType::ZOOM are not supported for CAMERA_ROVER_ANTENNA.");
        }
        else
        {
            static_assert(false, "Unknown eServoType for CAMERA_ROVER_ANTENNA.");
        }
    }
    else
    {
        static_assert(false, "Unsupported GLOBAL_DEVICE_ID or missing configuration.");
    }
}
#endif  // REV_0

#endif  // __CONFIG_LOCAL_HPP__
