#ifndef ROVER_LIB2_CONTROLLERS_CONTROLLER_HPP
#define ROVER_LIB2_CONTROLLERS_CONTROLLER_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <concepts>

namespace Controllers
{

    template<typename ImplT>
    concept Controller = requires(ImplT impl_)
    {
        // clang-format off
        { impl_.computeCommand(float{} /*input_*/, float{} /*target_*/) } -> std::same_as<float>;

        { impl_.reset() } -> std::same_as<void>;
        // clang-format on
    };

    class None
    {
      public:
        float computeCommand(float, float)
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void reset()
        {
            ASSERT_MSG("Interface");
        }
    };

}  // namespace Controllers

#endif  // ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
