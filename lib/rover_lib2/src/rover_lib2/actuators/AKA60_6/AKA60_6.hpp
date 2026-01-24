#include "rover_lib2/actuators/actuator.hpp"

namespace Actuators
{
    class AKA60_6
    {
        static constexpr float RAD_S_TO_RPM = 9.549296596425384F;

        AKA60_6()
        {

        }
        
        void init(){}

        void update()
        {
            ASSERT_MSG("Interface");
        }

        void setPosition(float /*pos_*/)
        {
            ASSERT_MSG("Not implemented");
        }

        float getPosition()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void setSpeed(float /*speed_*/)
        {
            ASSERT_MSG("Interface");
        }

        float getSpeed()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void setMaxSpeed(float /*max_speed_*/)
        {
            ASSERT_MSG("Interface");
        }

        void setJointLimit(std::optional<float> /*min_*/, std::optional<float> /*max_*/)
        {
            ASSERT_MSG("Interface");
        }
    };
}