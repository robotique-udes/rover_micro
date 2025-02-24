#ifndef CAN_DEVICE
#define CAN_DEVICE

#include "rover_can2/constant.hpp"
#include "rover_can2/can_driver.hpp"

namespace RoverCan2
{
    class CanDeviceBase
    {
      public:
        virtual void update() = 0;
        virtual constexpr RoverCan2::Constant::eDeviceId getID(void) = 0;
        virtual bool loadMessage(const CanDriver::sCanMsg msg_) = 0;
    };

    template<RoverCan2::Constant::eDeviceId CAN_ID, typename MSG_TYPE, typename CONTEXT_TYPE>
    class CanDevice : public CanDeviceBase
    {
      public:
        typedef void (*callback_t)(const MSG_TYPE& msg_, CONTEXT_TYPE context_);

        CanDevice(callback_t callbackFunc_, auto& context_): _callbackFunc(callbackFunc_), _context(context_) {}

        void update(void) override
        {
            (*_callbackFunc)(_msg, _context);
        }

        constexpr RoverCan2::Constant::eDeviceId getID(void) override
        {
            return CAN_ID;
        }

        bool loadMessage(const CanDriver::sCanMsg /*msg_*/)
        {
            // TODO: Process msg and call callback accordingly
            (*_callbackFunc)(_msg, _context);

            return false;
        }

      private:
        callback_t _callbackFunc;
        MSG_TYPE _msg;
        CONTEXT_TYPE& _context;
    };
}  // namespace RoverCan2

#endif  // CAN_DEVICE
