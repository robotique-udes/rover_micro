#include <Arduino.h>
#include "rover_helpers/helpers.hpp"
#include "string.h"
#include "math.h"

#define RX1PIN GPIO_NUM_12
#define TX1PIN GPIO_NUM_13
#define BUFFER_SIZE 255

#define NB_ELEMENTS 20
#define SIZE_VALUES 255

float getLatitude(char latData[], char latSign[]);
int splitData(const char *pGpsData);
float getLongitude(char longData[], char longSign[]);

float latitude;
float longitude;
float fixType;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX1PIN, TX1PIN);

 

  for(EVER)
  {
    char bufferGpsReceive[BUFFER_SIZE] = {0};
    uint8_t dataSize = Serial1.readBytesUntil('\n', bufferGpsReceive, BUFFER_SIZE - 1);

    if (dataSize > 0)
    {
      // Good Received data: $GNGGA,201427.00,4522.65273,N,07155.50339,W,2,12,0.71,282.0,M,-31.5,M,,0000*78
      bufferGpsReceive[dataSize] = '\0';
      const char *pGpsData = bufferGpsReceive;
      char gpsDataTest[BUFFER_SIZE] = {'$','G','N','G','G','A',',','2','0','1','4','2','7','.','0','0',',','4','5','2','2','.','6','5','2','7','3',',','N',',','0','7','1','5','5','.','5','0','3','3','9',',','W',',','2',',','1','2',',','0','.','7','1',',','2','8','2','.','0',',','M',',','-','3','1','.','5',',','M',',',',','0','0','0','0','*','7','8','\0'};
      const char *pGpsDataTest = gpsDataTest;
      splitData(pGpsData);
    }
  }
}

void loop(){}

int splitData(const char *pGpsData)
{
  char data[NB_ELEMENTS][SIZE_VALUES] = {0};
  uint8_t indexElements = 0;
  uint8_t indexChar = 0;
  bool valide = true;

  if (pGpsData[0] == '\n' || pGpsData[0] == '\0' || pGpsData[0] != '$')
  {
    valide = false;
    LOG(WARN, "Empty message or not formatted correctly");
  }
  else
  {
    uint16_t i = 0;
    for (; pGpsData[i] != '\n' && pGpsData[i] != '\0' && i < UINT8_MAX; i++)
    {
      if (pGpsData[i] == ',')
      {
        data[indexElements][indexChar + 1] = '\0';
        indexChar = 0;
        indexElements++;
      }
      else
      {
        data[indexElements][indexChar] = pGpsData[i];
        indexChar++;
      }
    }
  }
  
  char compareMsgType[] = "$GNGGA";
  if (strcmp(data[0], compareMsgType) == 0)
  {
    LOG(INFO, "Good format of received data: %s", pGpsData);
  }
  else{
    valide = false;
  }

  fixType = atoi(data[6]);
  if (fixType == 0)
  {
    valide = false;
    LOG(WARN, "Invalid messages");
  }

  if (valide)
  {
    if (data[2][0] != '\0')
    {
      latitude = getLatitude(data[2], data[3]);
      LOG(INFO, "Latitude: %f", latitude);
    }
    else{
      valide = false;
      LOG(WARN, "Empty messages");
    }

    if (data[4][0] != '\0')
    {
      longitude = getLongitude(data[4], data[5]);
      LOG(INFO, "Longitude: %f", longitude);
    }
    else{
      valide = false;
      LOG(WARN, "Empty messages");
    }
  }

  return 0;
}

float getLatitude(char latData[], char latSign[])
{
  float allDegrees = 0;
  float allMinutes = 0;
  uint8_t i = 0;
  int8_t iDeg = 1;
  int8_t iMin = 1;

  for (; latData[i] != '\0'; i++)
  {
    uint8_t temp = 0;
    if (latData[i] != '.' && latData[i] != '-')
    {
      temp = latData[i] - '0';
      if (iDeg >= 0)
      {
        
        allDegrees += temp * pow(10,iDeg);
        iDeg--;
      }
      else if (iDeg == -1)
      {
        allMinutes += temp * pow(10,iMin);
        iMin--;
      }
    }
  }
  float latitude = allDegrees + (allMinutes/60);
  if (latSign[0] == 'S')
  {
    latitude *= -1;
  }
  return latitude;
}

float getLongitude(char longData[], char longSign[])
{
  float allDegrees = 0;
  float allMinutes = 0;
  uint8_t i = 0;
  int8_t iDeg = 2;
  int8_t iMin = 1;

  for (; longData[i] != '\0'; i++)
  {
    uint8_t temp = 0;
    if (longData[i] != '.' && longData[i] != '-')
    {
      temp = longData[i] - '0';
      if (iDeg >= 0)
      {
        
        allDegrees += temp * pow(10,iDeg);
        iDeg--;
      }
      else if (iDeg == -1)
      {        
        allMinutes += temp * pow(10,iMin);
        iMin--;
      }
    }
  }

  float longitude = allDegrees + (allMinutes/60);
  if (longSign[0] == 'W')
  {
    longitude *= -1;
  }
  else if (longSign[0] == 'E')
  {
    longitude -= 360.0;
  }
  return longitude;
}