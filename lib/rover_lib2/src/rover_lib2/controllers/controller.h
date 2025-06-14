#ifndef ROVER_LIB2_CONTROLLERS_CONTROLLER_HPP
#define ROVER_LIB2_CONTROLLERS_CONTROLLER_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/macros.hpp"

namespace Controllers
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
        float computeCommand(float /*input_*/, float /*target_*/)
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void reset()
        {
            ASSERT_MSG("Interface");
        }
    };

    template<typename Impl_T>
    class Controller : public BaseT
    {
      private:
        friend Impl_T;
        Controller() = default;

      public:
        float computeCommand(float input_, float target_)
        {
            return static_cast<Impl_T*>(this)->_computeCommand(input_, target_);
        }

        void reset()
        {
            static_cast<Impl_T*>(this)->_reset();
        }
    };

}  // namespace Controllers

#endif  // ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
