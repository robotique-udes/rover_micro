#include <gtest/gtest.h>

#if defined(TEST_NATIVE)
//#include "native/helpers/macros.hpp"
// #include "native/helpers/chrono.hpp"
// #include "native/helpers/circular_buffer.hpp"
#include "native/helpers/moving_average.hpp"
// #include "native/helpers/watchdog.hpp"
// #include "native/helpers/loop_timer.hpp"
// #include "native/helpers/one_shot_timer.hpp"

// #include "native/rover_can2/helpers.hpp"
// #include "native/rover_can2/msg.hpp"
// #include "native/rover_can2/subscriber_standalone.hpp"
// #include "native/rover_can2/subscriber_member.hpp"
// #include "native/rover_can2/publisher.hpp"
// #include "native/rover_can2/device.hpp"
// #include "native/rover_can2/device_member.hpp"
// #include "native/rover_can2/manager.hpp"
// #include "native/rover_can2/integration_tests.hpp"

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif  // defined(TEST_NATIVE)
