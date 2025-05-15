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
    int c = GNSSSerial->read();
    //LOG_INFO(Logger::Nodes::Main, "Charactere recu: %c", static_cast<char>(c));
    
    if (c == '\n')
    {
      if (index > 0)
      {
        sentenceBuffer[index] = '\0'; // Ensure null-termination

        //LOG_INFO(Logger::Nodes::Main, "%s", sentenceBuffer.data());
        if (sentenceBuffer[0] == '$' || sentenceBuffer[0] == '#')
        {
          parseMSG(sentenceBuffer, index);
        }
      }
      index = 0;
    }
    else if (c != '\r' && index < MAX_SENTENCE_LENGTH - 1)
    {
      sentenceBuffer[index++] = static_cast<char>(c);
      sentenceBuffer[index] = '\0'; // Keep null-terminated
    }
  }
}


void GNSSManager::parseMSG(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_) 
{
  if (length_ < 6 || (sentence_[0] != '$' && sentence_[0] != '#')) 
  {
    LOG_ERROR(Logger::Nodes::Main, "Le message reçu est tout cassé bozo: lenght: %d, sentence 0: %c", length_, sentence_)
    return;
  }

  const char* tokens[32] = { nullptr };
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

  if ((strncmp(buffer, "$GPGGA", 6) == 0 || strncmp(buffer, "$GNGGA", 6) == 0) && tokenCount >= 8) 
  {
    currentData_.latitude = convertToDecimalDegrees(tokens[2], tokens[3][0]);
    currentData_.longitude = convertToDecimalDegrees(tokens[4], tokens[5][0]);
    currentData_.fixQuality = atoi(tokens[6]);
    currentData_.satellites = atoi(tokens[7]);
  }
  if (strncmp(buffer, "#UNIHEADING", 6) == 0 && tokenCount >= 8) 
  {
    updateHeadingFilter(atof(tokens[12]));
    currentData_.headingDeg = getFilteredHeading();
  }
}


float GNSSManager::convertToDecimalDegrees(const char* nmeaCoord_, char direction_) 
{
  if (!nmeaCoord_ || strlen(nmeaCoord_) < 6)
  {
  return 0.0;
  }

  float degMin = atof(nmeaCoord_);
  int degrees = static_cast<int>(degMin / 100);
  float minutes = degMin - degrees * 100;
  float decimal = degrees + minutes / 60.0;

  if (direction_ == 'S' || direction_ == 'W') decimal *= -1;
  return decimal;
}


void GNSSManager::updateHeadingFilter(float newHeadingDeg) {
  float newRad = newHeadingDeg * DEG_TO_RAD;
  float oldRad = headingBuffer_[headingIndex_];

  // If buffer is not yet full, just grow it
  if (headingCount_ < HEADING_FILTER_WINDOW) {
      headingCount_++;
  } else {
      // Subtract oldest value from sum
      sinSum_ -= sin(oldRad);
      cosSum_ -= cos(oldRad);
  }

  // Replace value in ring buffer
  headingBuffer_[headingIndex_] = newRad;
  headingIndex_ = (headingIndex_ + 1) % HEADING_FILTER_WINDOW;

  // Add new value
  sinSum_ += sin(newRad);
  cosSum_ += cos(newRad);

  currentData_.headingDeg = getFilteredHeading();
}


float GNSSManager::getFilteredHeading(void) const {
  if (headingCount_ == 0) return 0.0;
  float avgRad = atan2(sinSum_ / headingCount_, cosSum_ / headingCount_);
  if (avgRad < 0) avgRad += 2 * PI;
  return avgRad * RAD_TO_DEG;
}


GNSSData GNSSManager::getData() const 
{
  return currentData_;
}


GNSSManager::~GNSSManager() 
{

}
