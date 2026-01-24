#ifndef JR_Actuator_HPP
#define JR_Actuator_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/IFX007T.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/filters/moving_average.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/filters/none.hpp"

#include <algorithm>


class JRActuator
{

};


#endif // JR_Actuator_HPP