/**
 * @file servo_calibration.hpp
 * @brief This file can be used to generate the ActuatorServoT::sTimingLimits struct for a specific motor easily.
 */

#include <Arduino.h>
#include "rover_lib2/LED/led_blinker.hpp"

#include "rover_can2/rover_can2.hpp"

#include "rover_lib2/actuators/actuator_servo.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/macros.hpp"

#warning Note: Currently, the "max_speed" mesurement isn't very accurate. Mesuring be hand is recommended

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_12;
constexpr gpio_num_t PIN_SERVO = GPIO_NUM_15;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

constexpr float PWM_FREQUENCY = 50.0F;          // Hz
constexpr gpio_num_t SERVO_GPIO = GPIO_NUM_15;  // Hz

enum CalibrationState
{
    CALIBRATE_MIN_MS,
    CONFIRM_MIN_MS,
    GET_MIN_ANGLE,
    CALIBRATE_MAX_MS,
    CONFIRM_MAX_MS,
    GET_MAX_ANGLE,
    SPEED_TEST_SETUP,
    SPEED_TEST_RUNNING,
    COMPLETE
};

CalibrationState currentState = CALIBRATE_MIN_MS;
float minMs = 0.0F;
float maxMs = 0.0F;
float minAngle = 0.0F;
float maxAngle = 0.0F;
float testMs = 0.0F;
float measuredSpeed = 0.0F;
unsigned long speedTestStartTime = 0;

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);
    led.init();

    PWMGenerators::MCPWMTimer timer(PWM_FREQUENCY, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM pwmGen(SERVO_GPIO, timer);
    pwmGen.init();
    pwmGen.setEnabled(true);

    LOG_INFO(Logger::Nodes::Main, "Welcome to the servo calibration and measurement tool");
    Serial.println("=== SERVO CALIBRATION ===");
    Serial.println("Step 1: Finding minimum pulse width");
    Serial.println("Enter pulse width in ms:");

    for (EVER)
    {
        led.update();
        pwmGen.update();

        if (Serial.available() > 0)
        {
            float userInput = Serial.parseFloat();
            // Clear any remaining characters in buffer
            while (Serial.available() > 0)
            {
                Serial.read();
            }

            switch (currentState)
            {
                case CALIBRATE_MIN_MS:
                    testMs = userInput;
                    Serial.print("Testing ");
                    Serial.print(testMs);
                    Serial.println(" ms");

                    pwmGen.setDutyCycle((100.0F * testMs) / (1'000'000.0F / PWM_FREQUENCY));

                    Serial.println("Is this the minimum position you want? (Enter 1 for yes, 0 to try another value):");
                    currentState = CONFIRM_MIN_MS;
                    break;

                case CONFIRM_MIN_MS:
                    if (userInput == 1.0F)
                    {
                        minMs = testMs;
                        Serial.println("Minimum pulse width saved!");
                        Serial.println("What angle (in degrees) does this position represent?");
                        currentState = GET_MIN_ANGLE;
                    }
                    else
                    {
                        Serial.println("Enter a new pulse width in ms:");
                        currentState = CALIBRATE_MIN_MS;
                    }
                    break;

                case GET_MIN_ANGLE:
                    minAngle = userInput;
                    Serial.print("Minimum angle set to ");
                    Serial.print(minAngle);
                    Serial.println(" degrees");
                    Serial.println();
                    Serial.println("Step 2: Finding maximum pulse width");
                    Serial.println("Enter pulse width in ms:");
                    currentState = CALIBRATE_MAX_MS;
                    break;

                case CALIBRATE_MAX_MS:
                    testMs = userInput;
                    Serial.print("Testing ");
                    Serial.print(testMs);
                    Serial.println(" ms");

                    pwmGen.setDutyCycle((100.0F * testMs) / (1'000'000.0F / PWM_FREQUENCY));

                    Serial.println("Is this the maximum position you want? (Enter 1 for yes, 0 to try another value):");
                    currentState = CONFIRM_MAX_MS;
                    break;

                case CONFIRM_MAX_MS:
                    if (userInput == 1.0F)
                    {
                        maxMs = testMs;
                        Serial.println("Maximum pulse width saved!");
                        Serial.println("What angle (in degrees) does this position represent?");
                        currentState = GET_MAX_ANGLE;
                    }
                    else
                    {
                        Serial.println("Enter a new pulse width in ms:");
                        currentState = CALIBRATE_MAX_MS;
                    }
                    break;

                case GET_MAX_ANGLE:
                    maxAngle = userInput;
                    Serial.print("Maximum angle set to ");
                    Serial.print(maxAngle);
                    Serial.println(" degrees");
                    Serial.println();
                    Serial.println("Step 3: Speed measurement");
                    Serial.println("Positioning servo to minimum position...");
                    Serial.flush();

                    // Move servo to minimum position
                    pwmGen.setDutyCycle((100.0F * minMs) / (1'000'000.0F / PWM_FREQUENCY));
                    delay(5000);  // Wait for servo to reach position

                    Serial.println("Servo is now at minimum position.");
                    Serial.println("Press any key xand ENTER to start the speed test (servo will move to max position):");
                    currentState = SPEED_TEST_SETUP;
                    break;

                case SPEED_TEST_SETUP:
                    Serial.println("Starting speed test...");
                    Serial.println("Timer started! Press any key and ENTER when servo reaches maximum position:");

                    // Start the movement to maximum position
                    pwmGen.setDutyCycle((100.0F * maxMs) / (1'000'000.0F / PWM_FREQUENCY));
                    speedTestStartTime = millis();
                    currentState = SPEED_TEST_RUNNING;
                    break;

                case SPEED_TEST_RUNNING:
                {
                    unsigned long elapsedTime = millis() - speedTestStartTime;
                    float elapsedTimeSeconds = elapsedTime / 1000.0F;

                    // Calculate angular displacement in radians
                    float angularDisplacement = abs(maxAngle - minAngle) * (PI / 180.0F);  // Convert degrees to radians

                    // Calculate speed in rad/s
                    measuredSpeed = angularDisplacement / elapsedTimeSeconds;

                    Serial.print("Movement completed in ");
                    Serial.print(elapsedTimeSeconds);
                    Serial.println(" seconds");
                    Serial.print("Angular displacement: ");
                    Serial.print(angularDisplacement);
                    Serial.println(" radians");
                    Serial.print("Measured speed: ");
                    Serial.print(measuredSpeed);
                    Serial.println(" rad/s");
                    Serial.println();
                    Serial.println("=== CALIBRATION COMPLETE ===");
                    Serial.println("Copy and paste this struct into your code:");
                    Serial.println();
                    Serial.println("struct sTimingLimits");
                    Serial.println("{");
                    Serial.print("    float frequency = ");
                    Serial.print(PWM_FREQUENCY);
                    Serial.println("F;");
                    Serial.print("    float minMs = ");
                    Serial.print(minMs);
                    Serial.println("F;");
                    Serial.print("    float maxMs = ");
                    Serial.print(maxMs);
                    Serial.println("F;");
                    Serial.print("    float minPosition = ");
                    Serial.print(minAngle * DEG_TO_RAD);
                    Serial.println("F;");
                    Serial.print("    float maxPosition = ");
                    Serial.print(maxAngle * DEG_TO_RAD);
                    Serial.println("F;");
                    Serial.print("    float maxSpeed = ");
                    Serial.print(measuredSpeed);
                    Serial.println("F;  // Measured speed in rad/s");
                    Serial.println("};");
                    Serial.println();
                    Serial.println("Calibration complete! Reset to calibrate again.");
                    currentState = COMPLETE;
                }
                break;

                case COMPLETE:
                    Serial.println("Calibration already complete. Reset to calibrate again.");
                    break;
            }
        }
    }
}

void loop() {}
