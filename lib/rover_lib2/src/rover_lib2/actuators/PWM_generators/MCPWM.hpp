#ifndef MCPWM_HPP
#define MCPWM_HPP

#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM_timer.hpp"

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

DEFINE_LOG_NODE(MCPWM, Logger::eNodeState::ON);

namespace PWMGenerators
{
    class MCPWM : public PWMGenerator<MCPWM>
    {
        static constexpr int DEFAULT_INTERUPT_PRIORITY = MCPWMTimer::DEFAULT_INTERUPT_PRIORITY;

      public:
        enum class ePinPullMode
        {
            FLOATING = 0,
            PULL_DOWN,
            PULL_UP,
            PULL_DOWN_UP  // Default value
        };

        enum class ePinOutputMode : bool
        {
            ACTIVE_HIGH = false,
            ACTIVE_LOW = true,
        };

        MCPWM(gpio_num_t io_,
              MCPWMTimer& timer_,
              ePinOutputMode outputMode_ = ePinOutputMode::ACTIVE_HIGH,
              ePinPullMode pinPullMode_ = ePinPullMode::FLOATING):
            _timer(timer_),
            _duty(0.0F)
        {
            mcpwm_comparator_config_t comparatorConfig = {
                .intr_priority = DEFAULT_INTERUPT_PRIORITY,
                .flags = {.update_cmp_on_tez = true, .update_cmp_on_tep = false, .update_cmp_on_sync = false},
            };
            _timer.createComparator(comparatorConfig, _comparatorH);

            gpio_reset_pin(io_);
            mcpwm_generator_config_t generatorConfig = {
                .gen_gpio_num = io_,
                .flags = {.invert_pwm = TO_UNDERLYING(outputMode_),
                          .io_loop_back = false,
                          .io_od_mode = false,
                          .pull_up = (pinPullMode_ == ePinPullMode::PULL_UP || pinPullMode_ == ePinPullMode::PULL_DOWN_UP),
                          .pull_down = (pinPullMode_ == ePinPullMode::PULL_DOWN || pinPullMode_ == ePinPullMode::PULL_DOWN_UP)}};
            _timer.createGenerator(generatorConfig, _generatorH);

            esp_err_t retVal = mcpwm_generator_set_action_on_timer_event(
                _generatorH,
                mcpwm_gen_timer_event_action_t{.direction = mcpwm_timer_direction_t::MCPWM_TIMER_DIRECTION_UP,
                                               .event = mcpwm_timer_event_t::MCPWM_TIMER_EVENT_EMPTY,
                                               .action = mcpwm_generator_action_t::MCPWM_GEN_ACTION_HIGH});
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_generator_set_action_on_timer_event() failed with %u", retVal);

            retVal = mcpwm_generator_set_action_on_compare_event(
                _generatorH,
                mcpwm_gen_compare_event_action_t{.direction = mcpwm_timer_direction_t::MCPWM_TIMER_DIRECTION_UP,
                                                 .comparator = _comparatorH,
                                                 .action = mcpwm_generator_action_t::MCPWM_GEN_ACTION_LOW});
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_generator_set_action_on_timer_event() failed with %u", retVal);

            this->_setDutyCycle(_duty);
            _timer.enable();
        }

        void __init(void) {}

        void __update(void) {}

        void _setDutyCycle(float duty_)
        {
            _duty = duty_;
            uint32_t activePeriodCtn = _timer.dutyToTickCtn(duty_);
            esp_err_t retval = mcpwm_comparator_set_compare_value(_comparatorH, activePeriodCtn);
            ASSERT_COND_MSG_ARGS(retval == ESP_OK, "mcpwm_comparator_set_compare_value(0x%p, %u)", _comparatorH, activePeriodCtn);
        }

        float _getDutyCycle(void)
        {
            return _duty;
        }

        /**
         * @brief Not supported, fails assertion
         *
         * @param duty_
         */
        void _setFrequency(float /*frequency_*/)
        {
            ASSERT_MSG("Changing frequency is not supported by MCPWM driver");
        }

        float _getFrequency(void)
        {
            return _timer.getFrequency();
        }

      private:
        MCPWMTimer& _timer;
        float _duty;

        mcpwm_cmpr_handle_t _comparatorH = nullptr;
        mcpwm_gen_handle_t _generatorH = nullptr;
    };
}  // namespace PWMGenerators

#endif  // MCPWM_HPP
