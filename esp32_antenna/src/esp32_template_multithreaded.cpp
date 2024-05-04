#include <Arduino.h>

#if !defined(ESP32)
#error CPU not supported
#endif

#include <std_msgs/msg/int32.h>
#include "rover_msgs/msg/antenna_cmd.h"
#include <std_msgs/msg/empty.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "rover_helpers/helpers.hpp"
#include "rover_msgs/msg/gps.h"
#include "bn880.h"

#include <semaphore>

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

#define PUL 27
#define DIR 26
#define EN 25

#define MICRO_STEPS 800
#define RATIO_MOTOR 99.51

#define HEARTBEAT_HZ 4

#define RX1PIN 16
#define TX1PIN 17
#define BUFFER_SIZE 255

#define NB_ELEMENTS 20
#define SIZE_ELEMENTS 255

float getLatitude(char latData[SIZE_ELEMENTS], char latSign[SIZE_ELEMENTS]);
void splitData(const char pGpsData[BUFFER_SIZE]);
float getLongitude(char longData[SIZE_ELEMENTS], char longSign[SIZE_ELEMENTS]);

float latitude;
float longitude;
uint8_t fixType;

// 800 micro-step/tour * ratio de la gearbox

// =============================================================================
//  This project can be used as a starting template for ESP32 microROS project
//
//  Logging:
//      Since microROS is talking over serial, you can't directly use
//      Serial.print to print. Instead use the LOG() macro ex:
//          LOG(INFO, "some_text %i", some_int);
//
//      To see the log in a terminal, run this cmd in a terminal:
//          ros2 launch rover_helper terminal_logger.launch.py
//
//      If the node is connected with a micro_ros_agent, it will output in the
//      terminal. If, for some reasons, your node isn't connecting, the output
//      will be printed in a terminal at the specified baud in Serial.begin().
//          **TODO**: Add macro to change the LOG macro stream
//
//      The logging level setting can be changed in the platformio.ini file by
//      changing the following entry: '-D LOGGER_LOWEST_LEVEL=10' to the level
//      you want (10, 20, 30, 40, 50)
//
//      Limitations:
//          Not thread safe, don't use in different threads, instead, share a
//          buffer between both core and LOG from the other core.
//
//  RoverMicroRosLib:
//      You can use this in house library to help writing node faster and safer
//      If you have any question with it you can contact the maintainer in the
//      lib_rover/library.properties file.
//      Current limitiation can be found in each header files
//
// =============================================================================

// Function forward declarations
void microRosLoop(void *pvParameters);
void cbGoal(const void *msg_);
void cbSubHeartbeat(const void *msg_);

// Global objects
RoverMicroRosLib::Subscriber subGoal = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, AntennaCmd), "/base/antenna/cmd/out/goal", cbGoal);
RoverMicroRosLib::Subscriber subHeartbeat = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/base/heartbeat", cbSubHeartbeat);
RoverMicroRosLib::Publisher pubGps = RoverMicroRosLib::Publisher(ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, Gps), "/pub_test_topic");
RoverHelpers::Timer<unsigned long, micros> timerStepper(200ul);
RoverHelpers::Timer<unsigned long, millis> timerHeartbeat(500ul);
RoverHelpers::Timer<unsigned long, millis> timerGps(1ul);
bool root_heartbeat_state = false;
bool state_heartbeat = false;
bool motor_status = false;
bool stepper_direction;
bn880 gps880(Serial2);

// Creating mutex
SemaphoreHandle_t xSemaphore = NULL;

void setup()
{
  Serial.begin(115200);
  pinMode(PUL, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);

  Serial1.begin(9600, SERIAL_8N1, RX1PIN, TX1PIN);

  xTaskCreatePinnedToCore(microRosLoop, "", 8192, NULL, 1, NULL, 0);

  xSemaphore = xSemaphoreCreateMutex();

  uint8_t motorStep = HIGH;

  digitalWrite(EN, LOW);
  digitalWrite(DIR, HIGH);

  // gps880.start();

  for (EVER)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            state_heartbeat = true;
        }
        else
        {
            state_heartbeat = false;
        }

        if (timer_heartbeat.isDone())
        {
            if (timer.isDone() && motor_status)
            {
                motorStep = motorStep == HIGH ? LOW : HIGH;
                digitalWrite(DIR, stepper_direction);
                digitalWrite(PUL, motorStep);
            }
            xSemaphoreGive(xSemaphore);
        }
    }
  }
}

void microRosLoop(void *pvParameters)
{
  set_microros_serial_transports(Serial);

  RoverMicroRosLib::Node<1u, 2u, 3u> node(NAME_NS, NAME_NODE);
  node.init();

  node.addSubscriber(&subHeartbeat);
  node.addSubscriber(&subGoal);
  node.addPublisher(&pubGps);

  for (EVER)
  {
    node.spinSome(RCL_MS_TO_NS(0UL));

    if (timerGps.isDone())
    {
      char bufferGpsReceive[BUFFER_SIZE] = {0};
      uint8_t dataSize = Serial1.readBytesUntil('\n', bufferGpsReceive, BUFFER_SIZE);
      if (dataSize > 0)
      {
        bufferGpsReceive[dataSize] = '\0';
        const char *pGpsData = bufferGpsReceive;
        splitData(pGpsData);
        rover_msgs__msg__Gps msg;
        msg.latitude = latitude;
        msg.longitude = longitude;
        pubGps.publish(&msg);
      }
    }
  }
}

void cbGoal(const void *msg_)
{
  unsigned long step_timer;
  const rover_msgs__msg__AntennaCmd *msg = (rover_msgs__msg__AntennaCmd *)msg_;
  // LOG(INFO, "Received: %f", msg->speed);

  if (timerHeartbeat.isDone())
  {
    root_heartbeat_state = false;
  }

  if (msg->speed != 0.0f && root_heartbeat_state)
  {
    step_timer = 2 * PI * 1e6 / (MICRO_STEPS * abs(msg->speed) * RATIO_MOTOR);
    // LOG(INFO, "Received: %i", step_timer);

    if (xSemaphoreTake(xSemaphore, (TickType_t)100) == pdTRUE)
    {
      if (msg->speed < 0)
      {
        stepper_direction = false;
      }
      else
      {
        stepper_direction = true;
      }

      motor_status = true;
      timerStepper.updateInterval(step_timer);
      digitalWrite(EN, LOW);
      xSemaphoreGive(xSemaphore);
    }
  }
  else if (msg->speed == 0.0f)
  {
    timerStepper.updateInterval(200);
    motor_status = false;
    digitalWrite(EN, HIGH);
  }
  else
  {
    motor_status = false;
  }
}

void cbSubHeartbeat(const void *msg_)
{
  root_heartbeat_state = true;
  timerHeartbeat.init(500);
}

void splitData(const char pGpsData[BUFFER_SIZE])
{
  char data[NB_ELEMENTS][SIZE_ELEMENTS] = {0};
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
    for (uint8_t i = 0; pGpsData[i] != '\n' && pGpsData[i] != '\0' && i < BUFFER_SIZE && indexElements + 1 < NB_ELEMENTS && indexChar + 1 < SIZE_ELEMENTS; i++)
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
    // LOG(INFO, "Good format of received data: %s", pGpsData);
    fixType = (uint8_t)atoi(data[6]);
    if (fixType == 0u)
    {
      valide = false;
      LOG(WARN, "Invalid signal, no position available");
    }
    else
    {
      LOG(INFO, "Good format of received data: %s", pGpsData);
    }
  }
  else
  {
    valide = false;
  }

  if (valide)
  {
    if (data[2][0] != '\0')
    {
      latitude = getLatitude(data[2], data[3]);
      LOG(INFO, "Latitude: %f", latitude);
    }
    else
    {
      valide = false;
      LOG(WARN, "Empty messages");
    }

    if (data[4][0] != '\0')
    {
      longitude = getLongitude(data[4], data[5]);
      LOG(INFO, "Longitude: %f", longitude);
    }
    else
    {
      valide = false;
      LOG(WARN, "Empty messages");
    }
  }
}

float getLatitude(char latData[SIZE_ELEMENTS], char latSign[SIZE_ELEMENTS])
{
  float allDegrees = 0.0f;
  float allMinutes = 0.0f;
  int8_t iDeg = 1;
  int8_t iMin = 1;

  for (uint8_t i = 0; latData[i] != '\0' && i < SIZE_ELEMENTS; i++)
  {
    uint8_t temp = 0;
    if (latData[i] != '.' && latData[i] != '-')
    {
      temp = latData[i] - '0';
      if (iDeg >= 0)
      {

        allDegrees += temp * pow(10, iDeg);
        iDeg--;
      }
      else if (iDeg == -1)
      {
        allMinutes += temp * pow(10, iMin);
        iMin--;
      }
    }
  }
  float latitude = allDegrees + (allMinutes / 60.0f);
  if (latSign[0] == 'S')
  {
    latitude *= -1.0f;
  }
  return latitude;
}

float getLongitude(char longData[SIZE_ELEMENTS], char longSign[SIZE_ELEMENTS])
{
  float allDegrees = 0.0f;
  float allMinutes = 0.0f;
  int8_t iDeg = 2;
  int8_t iMin = 1;

  for (uint8_t i = 0; longData[i] != '\0' && i < SIZE_ELEMENTS; i++)
  {
    uint8_t temp = 0;
    if (longData[i] != '.' && longData[i] != '-')
    {
      temp = longData[i] - '0';
      if (iDeg >= 0)
      {

        allDegrees += temp * pow(10, iDeg);
        iDeg--;
      }
      else if (iDeg == -1)
      {
        allMinutes += temp * pow(10, iMin);
        iMin--;
      }
    }
  }

  float longitude = allDegrees + (allMinutes / 60);
  if (longSign[0] == 'W')
  {
    longitude *= -1.0f;
  }
  else if (longSign[0] == 'E')
  {
    longitude -= 360.0f;
  }
  return longitude;
}

// Do not use when using FreeRTOS.
void loop() {}
