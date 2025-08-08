/**
 * @file device_config.hpp
 *
 * @brief This file should be use to change parameter before uploading into a new device of the same type.
 */

#ifndef DEVICE_CONFIG_HPP
#define DEVICE_CONFIG_HPP

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
#define PCB_ROVER_SERVO_CONTROLLER_REV_0

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions after this necessary between uploads
// ===============================================================================================================================

#if defined(PCB_ROVER_SERVO_CONTROLLER_REV_0)
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
    if constexpr (DEVICE_ID == RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN)
    {
        if constexpr (servoType_ == eServoType::PAN)
        {
            return Actuators::ServoT::sTimingConfig{
                .frequency = 50.0F,
                .minMs = 500.0F,
                .maxMs = 2500.0F,
                .minPosition = 0.0F,
                .maxPosition = std::numbers::pi_v<float> * 1.5F,
                .maxSpeed = 2.41F,
                .alignedPosition = (std::numbers::pi_v<float> * 1.5F - 0.0F) / 5.0F, // (maxPos - minPos) / 5.0F
            };
        }
        else
        {
            static_assert(false, "Not supported");
        }
    }    

    else if constexpr (DEVICE_ID == RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)
    {
        if constexpr (servoType_ == eServoType::PAN)
        {
            return Actuators::ServoT::sTimingConfig{
                .frequency = 50.0F,
                .minMs = 545.0F,
                .maxMs = 2480.0F,
                .minPosition = 0.0F,
                .maxPosition = std::numbers::pi_v<float> * 2.0F,
                .maxSpeed = 0.76F,
                .alignedPosition = (std::numbers::pi_v<float> * 2.0F - 0.0F) / 2.0F, // (maxPos - minPos) / 2.0F
            };
        }
        else
        {
            static_assert(false, "Not supported");
        }
    }
    else
    {
        static_assert(false, "Not supported for specified DEVICE_ID or missing configuration.");
    }
}
#endif  // REV_0

#endif  // DEVICE_CONFIG_HPP
