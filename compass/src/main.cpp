#include <Arduino.h>

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/compass.hpp"

#include "rover_helpers/helpers.hpp"
#include "rover_helpers/log.hpp"


#define DEVICE_ID (uint16_t)RoverCanLib::Constant::eDeviceId::COMPASS
#define RX1_PIN GPIO_NUM_12
#define TX1_PIN GPIO_NUM_13
#define CAN_TX GPIO_NUM_47
#define CAN_RX GPIO_NUM_48

#define BUFFER_SIZE 11
#define FILTER_SIZE 1

uint8_t buffer[BUFFER_SIZE];
int bufferIndex = 0;

void noActions(RoverCanLib::CanBusManager *dontUse0_, const twai_message_t *dontUse1_);
void parseAnglePacket();
void calibrate();

float g_roll;
float g_pitch;
float g_yaw;

const float YAW_OFFSET = -10.0;

void setup() {
    delay(2500);
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, CAN_TX, CAN_RX, noActions, true);

    canBus.init();

    RoverCanLib::Msgs::Compass compassMsg;
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(1000);

    //calibrate();

    LOG(INFO, "Init done, starting loop!");

    for(;;)
    {
        //canBus.update();

        while (Serial1.available()) 
        {
            uint8_t c = Serial1.read();

            if (bufferIndex < BUFFER_SIZE) {
                buffer[bufferIndex++] = c;
            }

            if (bufferIndex == BUFFER_SIZE) 
            {
                if (buffer[0] == 0x55) {
                    switch (buffer[1]) {
                        case 0x51:
                            //parseAccelerationPacket();
                            break;
                        case 0x53:  // Angle output
                            parseAnglePacket();
                            break;
                        default:
                            //Serial.println("Unknown packet type");
                            break;
                    }
                }
                bufferIndex = 0;
            }
            
            /* 
            if (timerFeedback.isDone())
            {
                compassMsg.data.roll = g_roll;
                compassMsg.data.pitch = g_pitch;
                compassMsg.data.yaw = g_yaw;

                canBus.sendMsg(&compassMsg);
            }
            */
        }
    }
}

void loop() {}


void calibrate() {
    LOG(INFO, "Starting sensor calibration...");

    // Enter Magneto calibration mode
    Serial1.write(0xFF);
    Serial1.write(0xAA);
    Serial1.write(0x01);
    Serial1.write(0x02);
    Serial1.write(0x00);
    delay(2000); // Wait for calibration to complete

    // Exit calibration mode
    Serial1.write(0xFF);
    Serial1.write(0xAA);
    Serial1.write(0x01);
    Serial1.write(0x00);
    Serial1.write(0x00);

    /*
    // Save Config
    Serial1.write(0xFF);
    Serial1.write(0xAA);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    */

    LOG(INFO, "Sensor calibration completed.");
}

void parseAnglePacket() {
    int16_t roll = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t yaw = (int16_t)((buffer[7] << 8) | buffer[6]);
    uint8_t VL = buffer[8];
    uint8_t VH = buffer[9];
    uint8_t receivedSum = buffer[10];

    uint8_t sum = 0x55 + 0x53 + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + VL + VH;

    if (sum == receivedSum) {
        g_roll = (roll / 32768.0) * 180.0;
        g_pitch = (pitch / 32768.0) * 180.0;
        g_yaw = (yaw / 32768.0) * 180.0;

        

        g_yaw += YAW_OFFSET;
        if (g_yaw < 0) {
            g_yaw += 360.0;
        } else if (g_yaw >= 360.0) {
            g_yaw -= 360.0;
        }
    
        Serial.print(g_pitch);
        Serial.print(", ");
        Serial.print(g_roll);
        Serial.print(", ");
        Serial.println(g_yaw);
        
    } else {
        //Serial.println("Checksum error in angle packet");
    }
}

void noActions(RoverCanLib::CanBusManager *dontUse0_, const twai_message_t *dontUse1_)
{
    REMOVE_UNUSED(&dontUse0_);
    REMOVE_UNUSED(dontUse1_);

    return;
} 