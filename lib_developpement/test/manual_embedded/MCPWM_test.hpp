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
 *  lib_developpement
 *
 *  Backup all your manuel tests under:
 *  lib_developpement/test/manual_embedded
 */

 #include <Arduino.h>
 #include "rover_lib2/LED/led_blinker.hpp"
 #include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
 
 #include "rover_can2/rover_can2.hpp"
 
 constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;
 constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_9;
 
 DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
 
 void setup(void)
 {
     Serial.begin(115200);
 #if defined(DEBUG)
     delay(1000);
 #endif
 
     PWMGenerators::MCPWMTimer pwmTimer(0UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
     PWMGenerators::MCPWM ledCan(PIN_CAN_LED, pwmTimer, PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH, PWMGenerators::MCPWM::ePinPullMode::PULL_DOWN_UP);
     PWMGenerators::MCPWM ledUser(PIN_USER_LED, pwmTimer, PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH);
     ledCan.setDutyCycle(0.0F);
     ledUser.setDutyCycle(50.0F);
 
     OneShotTimer<uint64_t, Time::millis> timer(5'250ULL);
 
     LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
     for (EVER)
     {
         if (timer.isReady())
         {
             ASSERT();
         }
     }
 }
 
 void loop() {}
 