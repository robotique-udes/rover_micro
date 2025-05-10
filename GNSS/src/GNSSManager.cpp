#include "GNSSManager.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);


GNSSManager::GNSSManager(Stream *serial_)
{
  // Initialize HardwareSerial on UART2
  GNSSSerial = serial_;
  delay(100);
}


void GNSSManager::update(void)
{
  static std::array<char, MAX_SENTENCE_LENGTH> sentenceBuffer;
  static size_t index = 0;

  while (GNSSSerial->available()) 
  {
    char c = GNSSSerial->read();
    if (c == '\n') 
    {
      if (index > 0 && sentenceBuffer[0] == '$') 
      {
        parseNMEA(sentenceBuffer, index);
      }
      index = 0;
    } 
    else if (c != '\r' && index < MAX_SENTENCE_LENGTH - 1) 
    {
      sentenceBuffer[index++] = c;
      sentenceBuffer[index] = '\0'; // null-terminate for safety
    }
  }
}


void GNSSManager::parseNMEA(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_) 
{
  if (length_ < 6 || sentence_[0] != '$') 
  {
    LOG_ERROR(Logger::Nodes::Main, "Le message reçu est tout cassé bozo")
    return;
  }

  const char* tokens[15] = { nullptr };
  size_t tokenCount = 0;
  char* buffer = const_cast<char*>(sentence_.data()); // safe because we're working on a local copy

  tokens[tokenCount++] = buffer;
  for (size_t i = 0; i < length_ && tokenCount < 15; ++i) 
  {
    if (buffer[i] == ',') 
    {
      buffer[i] = '\0';
      if (i + 1 < length_) 
      {
        tokens[tokenCount++] = &buffer[i + 1];
      }
    }
  }

  if (strncmp(buffer, "$GPGGA", 6) == 0 && tokenCount >= 8) 
  {
    currentData_.latitude = convertToDecimalDegrees(tokens[2], tokens[3][0]);
    currentData_.longitude = convertToDecimalDegrees(tokens[4], tokens[5][0]);
    currentData_.fixQuality = atoi(tokens[6]);
    currentData_.satellites = atoi(tokens[7]);
  }
  else if (strncmp(buffer, "$GPRMC", 6) == 0 && tokenCount >= 9) 
  {
      currentData_.headingDeg = atof(tokens[8]);
  }
}


double GNSSManager::convertToDecimalDegrees(const char* nmeaCoord_, char direction_) 
{
  if (!nmeaCoord_ || strlen(nmeaCoord_) < 6)
  {
  LOG_ERROR(Logger::Nodes::Main, "Les Coords suck: pas assez de caractre")
  return 0.0;
  }

  double degMin = atof(nmeaCoord_);
  int degrees = static_cast<int>(degMin / 100);
  double minutes = degMin - degrees * 100;
  double decimal = degrees + minutes / 60.0;

  if (direction_ == 'S' || direction_ == 'W') decimal *= -1;
  return decimal;
}


GNSSData GNSSManager::getData() const 
{
  return currentData_;
}


GNSSManager::~GNSSManager() 
{

}
