#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <rover_lib2/rover_object.hpp>

template<typename Impl_T>
class Encoder : public RoverObject<Encoder<Impl_T>>
{
  private:
    friend Impl_T;
    Encoder(bool reversed_):
        _reversed(reversed_)
    {
    }

  public:
    void _init(void)
    {
        static_cast<Impl_T*>(this)->__init();
    }

    void _update(void)
    {
        static_cast<Impl_T*>(this)->__update();
    }

    bool dataIsValid(void)
    {
        return static_cast<Impl_T*>(this)->_dataIsValid();
    }

    float getPosition(void)
    {
        float posRaw = static_cast<Impl_T*>(this)->_getPosition();
        if (!_reversed)
        {
            return -posRaw;
        }
        else
        {
            return posRaw;
        }
    }

    float getSpeed(void)
    {
        float speedRaw = static_cast<Impl_T*>(this)->_getSpeed();
        if (!_reversed)
        {
            return -speedRaw;
        }
        else
        {
            return speedRaw;
        }
    }

    void calib(float offset_)
    {
#warning TODO: Constrain offset around 2PI
        float offset = _reversed ? -offset_ : offset_;
        static_cast<Impl_T*>(this)->_calib(offset);
    }

    bool _reversed;
};

#endif  // ENCODER_HPP
