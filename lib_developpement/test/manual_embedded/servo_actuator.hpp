/**
 * @file main.hpp
 * @brief Main for quick testing or library implementation.
 *
 * The following folder structure in vscode is recommended to make intellisense work in the lib folder
 * └── lib_developpement
 *   ├── .clang-format
 *   ├── .gitignore
 *   ├── include
 *   ├── .pio
 *   ├── platformio.ini
 *   ├── src
 *   ├── test
 *   └── .vscode
 * └── lib
 *   ├── .clang-format
 *   ├── lib_rover
 *   ├── rover_can2
 *   ├── rover_can_lib
 *   ├── rover_lib2
 *   └── .vscode
 *
 *  [WARNING] For intellisense to work, replace the default .vscode inside the lib folder with the one generated under
 * lib_developpement
 *
 * Backup all your manuel tests under:
 *  lib_developpement/test/manual_embedded
 */

 #include <Arduino.h>
 #include "rover_lib2/LED/led_blinker.hpp"
 
 #include "rover_can2/rover_can2.hpp"
 
 #include "rover_lib2/actuators/actuator_servo.hpp"
 #include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
 #include "rover_lib2/helpers/loop_timer.hpp"
 
 constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_12;
 constexpr gpio_num_t PIN_SERVO = GPIO_NUM_15;
 
 DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
 
 void setup(void)
 {
     Serial.begin(115200);
 #if defined(DEBUG)
     delay(1000);
 #endif
 
     LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);
     led.init();
 
     PWMGenerators::MCPWMTimer timer(50.0F, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
     PWMGenerators::MCPWM pwmGen(GPIO_NUM_15, timer);
     ActuatorServo<PWMGenerators::MCPWM> servo(ActuatorServoT::eModel::BILDA_TORQUE_FIVE_TURN, pwmGen, false);
     servo.init();
 
     LoopTimer<uint64_t, Time::micros> timerCmdChange(5000.0F);
     LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
     for (EVER)
     {
         led.update();
         servo.update();
 
         servo.setPosition(90.0F);
     }
 }
 
 void loop() {}
 