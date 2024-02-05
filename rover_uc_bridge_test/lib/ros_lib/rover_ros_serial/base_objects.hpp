#ifndef __BASE_OBJECTS_HPP__
#define __BASE_OBJECTS_HPP__

#include <stdint.h>
#include <cstring>

#include "Arduino.h"
#include "helpers/log.h"

namespace RoverRosSerial
{
    namespace Constant
    {
        constexpr uint8_t BEGIN = 128u;

        enum eHeaderType : uint8_t
        {
            notInitialised = 0,
            heartbeat = 129u,
            log = 130u,
            msg = 150u,
            srv = 200u
        };

        struct sHeader
        {
            uint16_t type;
            uint16_t length;
        };

        union uHeader
        {
            sHeader header;
            uint8_t data[sizeof(sHeader)];
        };
    }

    class Msg
    {
    public:
        Msg(HardwareSerial *serial_ = &Serial)
        {
            if (serial_ == NULL)
            {
                pSerial = &Serial;
            }

            pSerial = serial_;
        }

    protected:
        Constant::uHeader uHeader;

        virtual uint8_t *getSerializedHeader(void)
        {
            return uHeader.data;
        }

        virtual uint8_t getSerializedheaderSize(void)
        {
            return sizeof(uHeader.data);
        }

        virtual uint8_t *getSerializedData(void) = 0;
        virtual uint8_t getSerializedDataSize(void) = 0;
        virtual void sendMsg()
        {
            pSerial->write(Constant::BEGIN);
            pSerial->write(getSerializedHeader(), getSerializedheaderSize());

            // Only send msg if it's not empty
            uint8_t msgLength = getSerializedDataSize();
            if (msgLength > 0)
            {
                pSerial->write(getSerializedData(), getSerializedDataSize());
            }
        }

    private:
        HardwareSerial *pSerial = NULL;
    };
}
#endif // __BASE_OBJECTS_HPP__
