#include <Arduino.h>
#include "rover_helpers/helpers.hpp"
#include "string.h"
#include "math.h"

#define RX1PIN GPIO_NUM_12
#define TX1PIN GPIO_NUM_13
#define BUFFER_SIZE 255
#define LINE_TYPE GNGGA

#define NB_ELEMENTS 20
#define SIZE_VALUES 255

float getLatitude(char lat_data[], char lat_sign[]);
int splitData(const char *pgps_data);
float getLongitude(char long_data[], char long_sign[]);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX1PIN, TX1PIN);

  for(EVER)
  {
    char buffer_gps_receive[BUFFER_SIZE] = {0};
    uint8_t gps_data = Serial1.readBytesUntil('\n', buffer_gps_receive, BUFFER_SIZE - 1);

    if (gps_data > 0)
    {
      // Good Received data: $GNGGA,201427.00,4522.65273,N,07155.50339,W,2,12,0.71,282.0,M,-31.5,M,,0000*78
      buffer_gps_receive[gps_data] = '\0';
      const char *pgps_data = buffer_gps_receive;
      char gps_data_test[BUFFER_SIZE] = {'$','G','N','G','G','A',',','2','0','1','4','2','7','.','0','0',',','4','5','2','2','.','6','5','2','7','3',',','N',',','0','7','1','5','5','.','5','0','3','3','9',',','W',',','2',',','1','2',',','0','.','7','1',',','2','8','2','.','0',',','M',',','-','3','1','.','5',',','M',',',',','0','0','0','0','*','7','8','\0'};
      const char *pgps_data_test = gps_data_test;
      splitData(pgps_data);
    }
  }
}

void loop(){}

int splitData(const char *pgps_data)
{
  char data[NB_ELEMENTS][SIZE_VALUES] = {0};
  uint8_t index_elements = 0;
  uint8_t index_char = 0;
  bool valide = true;
  bool test = false;

  if (pgps_data[0] == '\n' || pgps_data[0] == '\0' || pgps_data[0] != '$')
  {
    valide = false;
    LOG(WARN, "Empty message or not formatted correctly");
  }
  else
  {
    uint16_t i = 0;
    for (; pgps_data[i] != '\n' && pgps_data[i] != '\0' && i < UINT8_MAX; i++)
    {
      if (pgps_data[i] == ',')
      {
        data[index_elements][index_char + 1] = '\0';
        index_char = 0;
        index_elements++;
      }
      else
      {
        data[index_elements][index_char] = pgps_data[i];
        index_char++;
      }
    }
  }
  
  char compare1[] = "$GNGGA";
  if (strcmp(data[0], compare1) == 0)
  {
    LOG(INFO, "Good format of received data: %s", pgps_data);
  }
  else{
    valide = false;
  }

  const char *compareFixType = "0";
  if (strcmp(data[6], compareFixType) == 0)
  {
    valide = false;
    LOG(WARN, "Invalid messages");
  }

  if (valide)
  {
    if (data[2][0] != '\0')
    {
      float latitude = getLatitude(data[2], data[3]);
      LOG(INFO, "Latitude: %f", latitude);
    }
    else{
      valide = false;
      LOG(WARN, "Empty messages");
    }

    if (data[4][0] != '\0')
    {
      float longitude = getLongitude(data[4], data[5]);
      LOG(INFO, "Longitude: %f", longitude);
    }
    else{
      valide = false;
      LOG(WARN, "Empty messages");
    }
  }

  return 0;
}

float getLatitude(char lat_data[], char lat_sign[])
{
  float allDegrees = 0;
  float allMinutes = 0;
  uint8_t i = 0;
  int8_t iDeg = 1;
  int8_t iMin = 1;
  bool negate = false;

  for (; lat_data[i] != '\0'; i++)
  {
    uint8_t temp = 0;
    if (lat_data[i] != '.' && lat_data[i] != '-')
    {
      temp = lat_data[i] - '0';
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
    else if (lat_data[i] == '-')
    {
      negate = true;
    }
  }
  float latitude = allDegrees + (allMinutes/60);
  if (lat_sign[0] == 'S')
  {
    latitude *= -1;
  }
  return latitude;
}

float getLongitude(char long_data[], char long_sign[])
{
  float allDegrees = 0;
  float allMinutes = 0;
  uint8_t i = 0;
  int8_t iDeg = 2;
  int8_t iMin = 1;
  bool negate = false;

  for (; long_data[i] != '\0'; i++)
  {
    uint8_t temp = 0;
    if (long_data[i] != '.' && long_data[i] != '-')
    {
      temp = long_data[i] - '0';
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
    else if (long_data[i] == '-')
    {
      negate = true;
    }
  }

  float longitude = allDegrees + (allMinutes/60);
  if (long_sign[0] == 'W')
  {
    longitude *= -1;
  }
  else if (long_sign[0] == 'E')
  {
    longitude -= 360.0;
  }
  return longitude;
}