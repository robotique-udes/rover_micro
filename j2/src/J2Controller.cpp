#include "J2Controller.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

// Checksum hash table
unsigned const short J2Controller::CRC16_TAB[]
    = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce,
       0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
       0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
       0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
       0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e,
       0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
       0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae,
       0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
       0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
       0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
       0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e,
       0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
       0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e,
       0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
       0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
       0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
       0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1,
       0x1ef0};

J2Controller::J2Controller(Stream* serial_)
{
    // LED and Button initialisation
    pinMode(LED_ERR, OUTPUT);
    pinMode(LED_BTLN, OUTPUT);
    pinMode(BUT_JOG_PLUS, INPUT_PULLUP);
    pinMode(BUT_JOG_MOINS, INPUT_PULLUP);

    // Initialize speed variable
    J2Controller::current_rpm = 0.0;
    J2Controller::target_rpm = 0.0;

    // Initialize HardwareSerial on UART2
    J2Controller::motorSerial = serial_;
    // SJ2Controller::motorSerial->begin(J2Controller::UART_BAUD_RATE, SERIAL_8N1, J2Controller::UART_RX_PIN,
    // J2Controller::UART_TX_PIN);

    // Serial.begin(115200); // Monitor output
    // Serial.println("UART initialized");
    delay(100);
}

void J2Controller::sendSpeedCommand(float rpm_)
{
    // Speed conversion to electrical RPM
    float erpm_f = rpm_ * 100.0 * 6.0 * 14.0;
    if (erpm_f > J2Controller::RATED_SPEED_ERPM)
    {
        erpm_f = J2Controller::RATED_SPEED_ERPM;
        LOG_WARN(Logger::Nodes::Main, "Trop vite bozo");
    }
    else if (erpm_f < -J2Controller::RATED_SPEED_ERPM)
    {
        erpm_f = -J2Controller::RATED_SPEED_ERPM;
        LOG_WARN(Logger::Nodes::Main, "Trop vite bozo");
    }

    // Convert to int32_t for proper signed byte handling
    int32_t erpm = static_cast<int32_t>(erpm_f);

    uint8_t buffer[10];

    buffer[0] = J2Controller::FRAME_HEAD;
    buffer[1] = 0x05;  // 1 byte command + 4 bytes speed
    buffer[2] = J2Controller::COMMAND_SET_RPM;
    buffer[3] = (erpm >> 24) & 0xFF;
    buffer[4] = (erpm >> 16) & 0xFF;
    buffer[5] = (erpm >> 8) & 0xFF;
    buffer[6] = erpm & 0xFF;

    // Calculate checksum
    uint16_t checksum = J2Controller::calculateCRC16(buffer + 2, 5);
    buffer[7] = (checksum >> 8) & 0xFF;
    buffer[8] = checksum & 0xFF;

    buffer[9] = J2Controller::FRAME_TAIL;

    J2Controller::motorSerial->write(buffer, 10);
    LOG_INFO(Logger::Nodes::Main, "Sent RPM Command: %f", rpm_);
}

unsigned short J2Controller::calculateCRC16(unsigned char* buf_, unsigned int len_)
{
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len_; i++)
    {
        cksum = J2Controller::CRC16_TAB[(((cksum >> 8) ^ *buf_++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}

void J2Controller::readMotorParameters(bool verbose)
{
    // Command to request motor parameters
    uint8_t buffer[6];
    buffer[0] = J2Controller::FRAME_HEAD;
    buffer[1] = 0x01;  // Data length: 1 byte
    buffer[2] = J2Controller::COMMAND_GET_VALUES;
    uint16_t checksum = J2Controller::calculateCRC16(buffer + 2, 1);
    buffer[3] = (checksum >> 8) & 0xFF;
    buffer[4] = checksum & 0xFF;
    buffer[5] = J2Controller::FRAME_TAIL;

    J2Controller::motorSerial->write(buffer, 6);
    LOG_INFO(Logger::Nodes::Main, "Sent motor parameter request");

    delay(10);  // Allow time for response

    if (J2Controller::motorSerial->available())
    {
        uint8_t response[100];
        int len = J2Controller::motorSerial->readBytes(response, sizeof(response));

        if (len < 4 || response[0] != J2Controller::FRAME_HEAD || response[len - 1] != J2Controller::FRAME_TAIL)
        {
            LOG_ERROR(Logger::Nodes::Main, "Invalid response");
            return;
        }

        uint16_t dataLen = response[1];
        if (dataLen + 5 != len)
        {
            LOG_ERROR(Logger::Nodes::Main, "Data length mismatch");
            return;
        }

        uint16_t crcReceived = (response[len - 3] << 8) | response[len - 2];
        uint16_t crcCalculated = J2Controller::calculateCRC16(response + 2, dataLen);
        if (crcReceived != crcCalculated)
        {
            LOG_ERROR(Logger::Nodes::Main, "CRC check failed");
            return;
        }

        int index = 3;
        J2Controller::param_mosTemp = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
        index += 2;
        J2Controller::param_motorTemp = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
        index += 2;
        J2Controller::param_outputCurrent
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 100.0;
        index += 4;
        J2Controller::param_inputCurrent
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 100.0;
        index += 4;
        J2Controller::param_idCurrent
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 100.0;
        index += 4;
        J2Controller::param_iqCurrent
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 100.0;
        index += 4;
        J2Controller::param_throttleValue = (int16_t)((response[index] << 8) | response[index + 1]) / 1000.0;
        index += 2;
        J2Controller::param_motorSpeed
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]);
        index += 4;
        J2Controller::param_inputVoltage = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
        index += 2;
        index += 24;  // Skip Reserved bytes
        J2Controller::param_motorStatusCode = response[index];
        index += 1;
        uint32_t positionData
            = (response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3];
        J2Controller::param_motorOuterLoopPosition = *((float*)&positionData);
        index += 4;
        J2Controller::param_motorIdNumber = response[index];
        index += 1;
        index += 6;  // Skip Temperature Reserved Value
        J2Controller::param_vdVoltage
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 1000.0;
        index += 4;
        J2Controller::param_vqVoltage
            = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3])
              / 1000.0;
        index += 4;
        if (index + 12 < len - 3)
        {  // Check if we have enough data (nouveaux messages)
            J2Controller::param_currentControlMode
                = (response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3];
            index += 4;
            uint32_t encoderAngleData
                = (response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3];
            J2Controller::param_encoderAngle = *((float*)&encoderAngleData);
            index += 4;
            uint32_t outerEncoderAngleData
                = (response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3];
            J2Controller::param_outerEncoderAngle = *((float*)&outerEncoderAngleData);
            index += 4;
        }

        if (verbose)
        {
            LOG_WARN(Logger::Nodes::Main, "MOS Temp: %.2f °C", J2Controller::param_mosTemp);
            LOG_WARN(Logger::Nodes::Main, "Motor Temp: %.2f °C", J2Controller::param_motorTemp);
            LOG_WARN(Logger::Nodes::Main, "Output Current: %.3f A", J2Controller::param_outputCurrent);
            LOG_WARN(Logger::Nodes::Main, "Input Current: %.3f A", J2Controller::param_inputCurrent);
            LOG_WARN(Logger::Nodes::Main, "Id Current: %.3f A", J2Controller::param_idCurrent);
            LOG_WARN(Logger::Nodes::Main, "Iq Current: %.3f A", J2Controller::param_iqCurrent);
            LOG_WARN(Logger::Nodes::Main, "Throttle Value: %.2f V", J2Controller::param_throttleValue);
            LOG_WARN(Logger::Nodes::Main, "Motor Speed: %.3f RPM", J2Controller::param_motorSpeed);
            LOG_WARN(Logger::Nodes::Main, "Input Voltage: %.2f V", J2Controller::param_inputVoltage);
            LOG_WARN(Logger::Nodes::Main, "Motor Status Code: %d", J2Controller::param_motorStatusCode);
            LOG_WARN(Logger::Nodes::Main, "Motor Outer Loop Position: %.2f Units", J2Controller::param_motorOuterLoopPosition);
            LOG_WARN(Logger::Nodes::Main, "Motor ID Number: %d", J2Controller::param_motorIdNumber);
            LOG_WARN(Logger::Nodes::Main, "Vd Voltage: %.2f V", J2Controller::param_vdVoltage);
            LOG_WARN(Logger::Nodes::Main, "Vq Voltage: %.2f V\n", J2Controller::param_vqVoltage);
            LOG_WARN(Logger::Nodes::Main, "Current Control Mode: %d", J2Controller::param_currentControlMode);
            LOG_WARN(Logger::Nodes::Main, "Encoder Angle: %.2f rad", J2Controller::param_encoderAngle);
            LOG_WARN(Logger::Nodes::Main, "Outer Encoder Angle: %.2f rad\n", J2Controller::param_outerEncoderAngle);
        }
    }
    else
    {
        LOG_ERROR(Logger::Nodes::Main, "No response from motor");
    }
}

void J2Controller::setSpeed(float rpm_)
{
    J2Controller::target_rpm = rpm_;
}

float J2Controller::getSpeed(void)
{
    return J2Controller::target_rpm;
}

void J2Controller::update(void)
{
    uint32_t now = millis();
    float dt = (now - J2Controller::last_ramp_time) / 1000.0f;
    J2Controller::last_ramp_time = now;

    float max_step = J2Controller::ramp_rate * dt;

    if (abs(J2Controller::target_rpm - J2Controller::current_rpm) <= max_step)
    {
        current_rpm = target_rpm;
    }
    else if (J2Controller::target_rpm > J2Controller::current_rpm)
    {
        J2Controller::current_rpm += max_step;
    }
    else
    {
        J2Controller::current_rpm -= max_step;
    }

    sendSpeedCommand(J2Controller::current_rpm);
}

bool J2Controller::isJogButtonPressed(bool plus_moins_)
{
    if (plus_moins_ == 0)
    {
        if (digitalRead(J2Controller::BUT_JOG_MOINS) == LOW)
        {
            return true;
        }
    }

    if (plus_moins_ == 1)
    {
        if (digitalRead(J2Controller::BUT_JOG_PLUS) == LOW)
        {
            return true;
        }
    }

    return false;
}

J2Controller::~J2Controller() {}
