#include "helpers/led_blinker.hpp"

LedBlinker::LedBlinker(gpio_num_t ledPin_, ledc_timer_t timerNumber_, ledc_channel_t channelNumber_)
{
    _ledPin = ledPin_;
    _timer = timerNumber_;
    _channel = channelNumber_;
}

LedBlinker::~LedBlinker()
{
    ledc_stop(LEDC_LOW_SPEED_MODE, _channel, 0u);
}

void LedBlinker::init(uint32_t frequencyHz_, float dutyCycle_)
{
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = _timer;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = 1;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = _channel;
    ledc_channel.timer_sel = _timer;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = _ledPin;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel_config(&ledc_channel);

    if (frequencyHz_ == 0)
    {
        LOG(WARN, "Invalid frequency, setting led off");
        this->setOff();
    }
    else
    {
        this->setBlink(frequencyHz_, dutyCycle_);
    }
}

void LedBlinker::setOn(void)
{
    this->setBlink(10000, 100.0f);
}

void LedBlinker::setOff(void)
{
    this->setBlink(10000, 0.0f);
}

void LedBlinker::setBlink(uint32_t frequencyHz_, float dutyCycle_)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, frequencyHz_);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, static_cast<uint32_t>(round(dutyCycle_/100.0f * 8096.0f)));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
}
