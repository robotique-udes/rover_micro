/*#include <Arduino.h>

// Checksum hash table
const unsigned short crc16_tab[] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 
    0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 
    0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 
    0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 
    0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 
    0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 
    0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 
    0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 
    0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 
    0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 
    0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

// Configure simple I/O
const int ledERR = 8;
const int ledBTLN = 9;
const int butJOGplus = 10;
const int butJOGmoins = 11;

// Configure UART pins
#define UART_TX_PIN 6
#define UART_RX_PIN 7
#define UART_BAUD_RATE 921600

// Motor Protocol Constants
#define FRAME_HEAD 0x02
#define FRAME_TAIL 0x03
#define COMMAND_SET_RPM 0x08

// Motor limtis
const int32_t ratedSpeed_erpm = 19572;
const int32_t maxSpeed_erpm = 26880; //no load

// Initialize HardwareSerial on UART2
HardwareSerial motorSerial(2); 

// Function Prototypes
void             sendSpeedCommand(int32_t rpm);
unsigned short   calculateCRC16(unsigned char *buf, unsigned int len);
void             readMotorParameters();


void setup()
{
  pinMode(ledERR, OUTPUT);
  pinMode(ledBTLN, OUTPUT);
  pinMode(butJOGplus, INPUT_PULLUP);
  pinMode(butJOGmoins, INPUT_PULLUP);

  Serial.begin(115200); // Monitor output
  motorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART initialized");
  delay(1000);
}


void loop()
{
  int32_t desiredRPM = 1; // Example RPM
  //sendSpeedCommand(desiredRPM);
  //delay(100);
  readMotorParameters();
}


void sendSpeedCommand(int32_t rpm)
{
  //Speed conversion to electrical RPM
  int32_t erpm = rpm*100*6*14;
  if (erpm > ratedSpeed_erpm) {
    erpm = 0;//ratedSpeed_erpm;
    Serial.println("Trop vite bozo");
  }

  uint8_t buffer[10]; // 10 bytes: Header-1, Length-1, Command-1, Data (4 bytes), Checksum (2 bytes), Tail-1

  buffer[0] = FRAME_HEAD;
  buffer[1] = 0x05; // Data Length (1 byte for command + 4 bytes for speed)
  buffer[2] = COMMAND_SET_RPM;
  buffer[3] = (erpm >> 24) & 0xFF;
  buffer[4] = (erpm >> 16) & 0xFF;
  buffer[5] = (erpm >> 8) & 0xFF;
  buffer[6] = erpm & 0xFF;

  // Calculate checksum
  uint16_t checksum = calculateCRC16(buffer + 2, 5);
  buffer[7] = (checksum >> 8) & 0xFF;
  buffer[8] = checksum & 0xFF;

  buffer[9] = FRAME_TAIL;

  motorSerial.write(buffer, 10);
  Serial.print("Sent RPM Command: ");
  Serial.println(rpm);
}


unsigned short calculateCRC16(unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
  cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}


void readMotorParameters() {
  // Command to request motor parameters
  uint8_t buffer[6] = {FRAME_HEAD, 0x01, 0x04, 0x40, 0x84, FRAME_TAIL};
  motorSerial.write(buffer, 6);
  Serial.println("Sent motor parameter request");

  delay(100); // Allow time for response
    
  if (motorSerial.available()) {
    uint8_t response[100];
    int len = motorSerial.readBytes(response, sizeof(response));

    if (len < 4 || response[0] != FRAME_HEAD || response[len - 1] != FRAME_TAIL) {
      Serial.println("Invalid response");
      return;
    }

    uint16_t dataLen = response[1];
    if (dataLen + 5 != len) {
      Serial.println("Data length mismatch");
      return;
    }

    uint16_t crcReceived = (response[len - 3] << 8) | response[len - 2];
    uint16_t crcCalculated = calculateCRC16(response + 2, dataLen);
    if (crcReceived != crcCalculated) {
      Serial.println("CRC check failed");
      return;
    }

    int index = 3;
    float mosTemp = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
    index += 2;
    float motorTemp = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
    index += 2;
    float outputCurrent = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 100.0;
    index += 4;
    float inputCurrent = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 100.0;
    index += 4;
    float idCurrent = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 100.0;
    index += 4;
    float iqCurrent = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 100.0;
    index += 4;
    float throttleValue = (int16_t)((response[index] << 8) | response[index + 1]) / 1000.0;
    index += 2;
    float motorSpeed = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]);
    index += 4;
    float inputVoltage = (int16_t)((response[index] << 8) | response[index + 1]) / 10.0;
    index += 2;
    index += 24; // Skip Reserved bytes
    uint8_t motorStatusCode = response[index];
    index += 1;
    float motorOuterLoopPosition = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 1000000.0;
    index += 4;
    uint8_t motorIdNumber = response[index];
    index += 1;
    index += 6; // Skip Temperature Reserved Value
    float vdVoltage = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 1000.0;
    index += 4;
    float vqVoltage = (int32_t)((response[index] << 24) | (response[index + 1] << 16) | (response[index + 2] << 8) | response[index + 3]) / 1000.0;
    index += 4;

    Serial.print("MOS Temp: "); Serial.print(mosTemp); Serial.println(" °C");
    Serial.print("Motor Temp: "); Serial.print(motorTemp); Serial.println(" °C");
    Serial.print("Output Current: "); Serial.print(outputCurrent); Serial.println(" A");
    Serial.print("Input Current: "); Serial.print(inputCurrent); Serial.println(" A");
    Serial.print("Id Current: "); Serial.print(idCurrent); Serial.println(" A");
    Serial.print("Iq Current: "); Serial.print(iqCurrent); Serial.println(" A");
    Serial.print("Throttle Value: "); Serial.print(throttleValue); Serial.println(" V");
    Serial.print("Motor Speed: "); Serial.print(motorSpeed); Serial.println(" RPM");
    Serial.print("Input Voltage: "); Serial.print(inputVoltage); Serial.println(" V");
    Serial.print("Motor Status Code: "); Serial.println(motorStatusCode);
    Serial.print("Motor Outer Loop Position: "); Serial.print(motorOuterLoopPosition); Serial.println(" Units");
    Serial.print("Motor ID Number: "); Serial.println(motorIdNumber);
    Serial.print("Vd Voltage: "); Serial.print(vdVoltage); Serial.println(" V");
    Serial.print("Vq Voltage: "); Serial.print(vqVoltage); Serial.println(" V\n");
  } else {
    Serial.println("No response from motor");
  }
}
*/


  /*  
    uint8_t buffer[64] = {0};
    int index = 0;
    
    // Wait for the frame head (0x02)
    while (true) {
        Serial.println("balls");
        motorSerial.write(param_cmd, 6);
        //if (motorSerial.available()) {
            if (motorSerial.read() == FRAME_HEAD) {
                break; // Exit the loop once we find the FRAME_HEAD
            }
        //}
    }
    // Read data length
    uint8_t dataLength = motorSerial.read();
    buffer[0] = dataLength;
    Serial.println(dataLength);
}*/

/*
if (digitalRead(butJOGplus) == LOW)
  {
    digitalWrite(ledERR, HIGH);
  } else {
    digitalWrite(ledERR, LOW);
  }

  if (digitalRead(butJOGmoins) == LOW)
  {
    digitalWrite(ledBTLN, HIGH);
  } else {
    digitalWrite(ledBTLN, LOW);
  }
*/

/*uint8_t buffer[7]; 
  buffer[0] = 0x02;
  buffer[1] = 0x05;
  buffer[2] = 0x08;
  buffer[3] = 0x00;
  buffer[4] = 0x00;
  buffer[5] = 0x03;
  buffer[6] = 0xE8;
  
  uint16_t checksum = calculateCRC16(buffer+2, 5);
  uint8_t dick = (checksum >> 8) & 0xFF;
  uint8_t balls = checksum & 0xFF;
  char marde[20];
  sprintf(marde, "Checksum High: %u (0x%02X)    Checksum Low: %u (0x%02X)", dick, dick, balls, balls);
  
  Serial.println(marde);
  */