#ifndef ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
#define ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP

#include <rover_lib2/rover_object.hpp>

namespace Encoders
{

    /**
     * @brief Shadow class for type validation and optional parameters
     * @attention [WARNING] Do not use directly as class or as pointer
     *
     */
    class BaseT
    {
      protected:
        BaseT() = default;

      public:
        void init(void)
        {
            ASSERT_MSG("Interface");
        }

        void update(void)
        {
            ASSERT_MSG("Interface");
        }

        bool dataIsValid(void)
        {
            ASSERT_MSG("Interface");
            return false;
        }

        float getPosition(void)
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        float getSpeed(void)
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void calib(float /*offset_*/)
        {
            ASSERT_MSG("Interface");
        }

        void setReversed(bool /*reverse_*/)
        {
            ASSERT_MSG("Interface");
        }
    };

    template<typename Impl_T>
    class Encoder : public RoverObject<Encoder<Impl_T>>,
                    public BaseT
    {
      private:
        friend Impl_T;
        Encoder() = default;

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
            return static_cast<Impl_T*>(this)->_getPosition();
        }

        float getSpeed(void)
        {
            return static_cast<Impl_T*>(this)->_getSpeed();
        }

        void calib(float offset_)
        {
            static_cast<Impl_T*>(this)->_calib(offset_);
        }

        void setReversed(bool reverse_)
        {
            static_cast<Impl_T*>(this)->_setReversed(reverse_);
        }
    };

}  // namespace Encoders
#endif  // ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
