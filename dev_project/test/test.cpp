#include <gtest/gtest.h>

#if defined(TEST_ON_DEVICE)
static_assert(false, "Not implemented yet")

#elif defined(TEST_NATIVE)
#include "native/helpers/chrono.hpp"
#include "native/helpers/circular_buffer.hpp"
#include "native/helpers/deref_array.hpp"
#include "native/helpers/watchdog.hpp"

#include "native/rover_can2/helpers.hpp"
#include "native/rover_can2/subscriber_standalone.hpp"
#include "native/rover_can2/subscriber_member.hpp"
#include "native/rover_can2/can_device.hpp"
#include "native/rover_can2/can_manager.hpp"

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#else // defined(TEST_ON_DEVICE)
static_assert(false, "Test type not selected")

#endif // defined(TEST_ON_DEVICE)
