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
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

#include "rover_can2/rover_can2.hpp"

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_NC;

constexpr gpio_num_t PIN_PHASE_A_EN = GPIO_NUM_8;
constexpr gpio_num_t PIN_PHASE_A_PWM = GPIO_NUM_7;

constexpr gpio_num_t PIN_PHASE_B_EN = GPIO_NUM_10;
constexpr gpio_num_t PIN_PHASE_B_PWM = GPIO_NUM_9;

constexpr gpio_num_t PIN_PHASE_C_EN = GPIO_NUM_12;
constexpr gpio_num_t PIN_PHASE_C_PWM = GPIO_NUM_11;

constexpr std::array FIXED_SINE_TABLE = {
    0.500000F, 0.508726F, 0.517450F, 0.526168F, 0.534878F, 0.543578F, 0.552264F, 0.560935F, 0.569587F, 0.578217F, 0.586824F,
    0.595404F, 0.603956F, 0.612476F, 0.620961F, 0.629410F, 0.637819F, 0.646186F, 0.654508F, 0.662784F, 0.671010F, 0.679184F,
    0.687303F, 0.695366F, 0.703368F, 0.711309F, 0.719186F, 0.726995F, 0.734736F, 0.742405F, 0.750000F, 0.757519F, 0.764960F,
    0.772320F, 0.779596F, 0.786788F, 0.793893F, 0.800908F, 0.807831F, 0.814660F, 0.821394F, 0.828030F, 0.834565F, 0.840999F,
    0.847329F, 0.853553F, 0.859670F, 0.865677F, 0.871572F, 0.877355F, 0.883022F, 0.888573F, 0.894005F, 0.899318F, 0.904508F,
    0.909576F, 0.914519F, 0.919335F, 0.924024F, 0.928584F, 0.933013F, 0.937310F, 0.941474F, 0.945503F, 0.949397F, 0.953154F,
    0.956773F, 0.960252F, 0.963592F, 0.966790F, 0.969846F, 0.972759F, 0.975528F, 0.978152F, 0.980631F, 0.982963F, 0.985148F,
    0.987185F, 0.989074F, 0.990814F, 0.992404F, 0.993844F, 0.995134F, 0.996273F, 0.997261F, 0.998097F, 0.998782F, 0.999315F,
    0.999695F, 0.999924F, 1.000000F, 0.999924F, 0.999695F, 0.999315F, 0.998782F, 0.998097F, 0.997261F, 0.996273F, 0.995134F,
    0.993844F, 0.992404F, 0.990814F, 0.989074F, 0.987185F, 0.985148F, 0.982963F, 0.980631F, 0.978152F, 0.975528F, 0.972759F,
    0.969846F, 0.966790F, 0.963592F, 0.960252F, 0.956773F, 0.953154F, 0.949397F, 0.945503F, 0.941474F, 0.937310F, 0.933013F,
    0.928584F, 0.924024F, 0.919335F, 0.914519F, 0.909576F, 0.904508F, 0.899318F, 0.894005F, 0.888573F, 0.883022F, 0.877355F,
    0.871572F, 0.865677F, 0.859670F, 0.853553F, 0.847329F, 0.840999F, 0.834565F, 0.828030F, 0.821394F, 0.814660F, 0.807831F,
    0.800908F, 0.793893F, 0.786788F, 0.779596F, 0.772320F, 0.764960F, 0.757519F, 0.750000F, 0.742405F, 0.734736F, 0.726995F,
    0.719186F, 0.711309F, 0.703368F, 0.695366F, 0.687303F, 0.679184F, 0.671010F, 0.662784F, 0.654508F, 0.646186F, 0.637819F,
    0.629410F, 0.620961F, 0.612476F, 0.603956F, 0.595404F, 0.586824F, 0.578217F, 0.569587F, 0.560935F, 0.552264F, 0.543578F,
    0.534878F, 0.526168F, 0.517450F, 0.508726F, 0.500000F, 0.491274F, 0.482550F, 0.473832F, 0.465122F, 0.456422F, 0.447736F,
    0.439065F, 0.430413F, 0.421783F, 0.413176F, 0.404596F, 0.396044F, 0.387524F, 0.379039F, 0.370590F, 0.362181F, 0.353814F,
    0.345492F, 0.337216F, 0.328990F, 0.320816F, 0.312697F, 0.304634F, 0.296632F, 0.288691F, 0.280814F, 0.273005F, 0.265264F,
    0.257595F, 0.250000F, 0.242481F, 0.235040F, 0.227680F, 0.220404F, 0.213212F, 0.206107F, 0.199092F, 0.192169F, 0.185340F,
    0.178606F, 0.171970F, 0.165435F, 0.159001F, 0.152671F, 0.146447F, 0.140330F, 0.134323F, 0.128428F, 0.122645F, 0.116978F,
    0.111427F, 0.105995F, 0.100682F, 0.095492F, 0.090424F, 0.085481F, 0.080665F, 0.075976F, 0.071416F, 0.066987F, 0.062690F,
    0.058526F, 0.054497F, 0.050603F, 0.046846F, 0.043227F, 0.039748F, 0.036408F, 0.033210F, 0.030154F, 0.027241F, 0.024472F,
    0.021848F, 0.019369F, 0.017037F, 0.014852F, 0.012815F, 0.010926F, 0.009186F, 0.007596F, 0.006156F, 0.004866F, 0.003727F,
    0.002739F, 0.001903F, 0.001218F, 0.000685F, 0.000305F, 0.000076F, 0.000000F, 0.000076F, 0.000305F, 0.000685F, 0.001218F,
    0.001903F, 0.002739F, 0.003727F, 0.004866F, 0.006156F, 0.007596F, 0.009186F, 0.010926F, 0.012815F, 0.014852F, 0.017037F,
    0.019369F, 0.021848F, 0.024472F, 0.027241F, 0.030154F, 0.033210F, 0.036408F, 0.039748F, 0.043227F, 0.046846F, 0.050603F,
    0.054497F, 0.058526F, 0.062690F, 0.066987F, 0.071416F, 0.075976F, 0.080665F, 0.085481F, 0.090424F, 0.095492F, 0.100682F,
    0.105995F, 0.111427F, 0.116978F, 0.122645F, 0.128428F, 0.134323F, 0.140330F, 0.146447F, 0.152671F, 0.159001F, 0.165435F,
    0.171970F, 0.178606F, 0.185340F, 0.192169F, 0.199092F, 0.206107F, 0.213212F, 0.220404F, 0.227680F, 0.235040F, 0.242481F,
    0.250000F, 0.257595F, 0.265264F, 0.273005F, 0.280814F, 0.288691F, 0.296632F, 0.304634F, 0.312697F, 0.320816F, 0.328990F,
    0.337216F, 0.345492F, 0.353814F, 0.362181F, 0.370590F, 0.379039F, 0.387524F, 0.396044F, 0.404596F, 0.413176F, 0.421783F,
    0.430413F, 0.439065F, 0.447736F, 0.456422F, 0.465122F, 0.473832F, 0.482550F, 0.491274F,
};

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    IO::DigitalOutput phaseA_en(PIN_PHASE_A_EN);
    phaseA_en.write(IO::eIOState::HIGH_);
    IO::DigitalOutput phaseB_en(PIN_PHASE_B_EN);
    phaseB_en.write(IO::eIOState::HIGH_);
    IO::DigitalOutput phaseC_en(PIN_PHASE_C_EN);
    phaseC_en.write(IO::eIOState::HIGH_);

    PWMGenerators::MCPWMTimer timerA(40'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM phaseA(PIN_PHASE_A_PWM,
                                timerA,
                                PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    PWMGenerators::MCPWMTimer timerB(40'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM phaseB(PIN_PHASE_B_PWM,
                                timerB,
                                PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    PWMGenerators::MCPWMTimer timerC(40'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM phaseC(PIN_PHASE_C_PWM,
                                timerC,
                                PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);

    LoopTimer<uint64_t, Time::micros> timerCoilUpdate(500);

    size_t currentIndex = 0;
    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        FIXED_SINE_TABLE.size();
        if (timerCoilUpdate.isReady())
        {
            size_t indexPhaseA = currentIndex;
            size_t indexPhaseB = currentIndex + 120;
            size_t indexPhaseC = currentIndex + 240;

            while (indexPhaseA > FIXED_SINE_TABLE.size())
            {
                indexPhaseA -= FIXED_SINE_TABLE.size();
            }

            while (indexPhaseB > FIXED_SINE_TABLE.size())
            {
                indexPhaseB -= FIXED_SINE_TABLE.size();
            }

            while (indexPhaseC > FIXED_SINE_TABLE.size())
            {
                indexPhaseC -= FIXED_SINE_TABLE.size();
            }

            phaseA.setDutyCycle(FIXED_SINE_TABLE[indexPhaseA] * 20.0F);  
            phaseB.setDutyCycle(FIXED_SINE_TABLE[indexPhaseB] * 20.0F);
            phaseC.setDutyCycle(FIXED_SINE_TABLE[indexPhaseC] * 20.0F);


            // LOG_INFO(Logger::Nodes::Main,
            //          "A: %f, B: %f, C: %f",
            //          phaseA.getDutyCycle(),
            //          phaseB.getDutyCycle(),
            //          phaseC.getDutyCycle());

            if (currentIndex++ >= FIXED_SINE_TABLE.size())
            {
                currentIndex = 0;
            }
        }

        led.update();
    }
}

void loop() {}
