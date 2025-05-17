#include "GNSSManager.hpp"

DEFINE_LOG_NODE(GNSS, Logger::eNodeState::OFF);


GNSSManager::GNSSManager(Stream *serial_)
{
  assert(serial_ != nullptr && "GNSSManager: serial_ pointer is null");
  _GNSSSerial = serial_;
  delay(100);
}


void GNSSManager::update(void)
{
  static std::array<char, MAX_SENTENCE_LENGTH> sentenceBuffer;
  static size_t index = 0;

  const size_t maxLoopCount = 1000;
  for (size_t i = 0; i < maxLoopCount && _GNSSSerial->available(); ++i)
  {
    int c = _GNSSSerial->read();
    if (c == -1)
    {
      continue;
    }
    char ch = static_cast<char>(c);
    //LOG_INFO(Logger::Nodes::GNSS, "Charactere recu: %c", ch);
    
    if (ch == '\n')
    {
      if (index > 0UL)
      {
        sentenceBuffer[index] = '\0';

        LOG_DEBUG(Logger::Nodes::GNSS, "%s", sentenceBuffer.data());
        if (sentenceBuffer[0] == '$' || sentenceBuffer[0] == '#')
        {
          parseMSG(sentenceBuffer, index);
        }
      }
      index = 0;
    }
    else if (ch != '\r' && index < sentenceBuffer.size() - 1)
    {
      sentenceBuffer[index++] = ch;
      sentenceBuffer[index] = '\0';
    }
  }
}


void GNSSManager::parseMSG(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_) 
{
  if (length_ < 6 || (sentence_[0] != '$' && sentence_[0] != '#')) 
  {
    LOG_ERROR(Logger::Nodes::GNSS, "Le message reçu est tout cassé bozo: lenght: %d, sentence 0: %c", length_, sentence_)
    return;
  }

  // Make a local mutable copy
  char buffer[MAX_SENTENCE_LENGTH] = {0};
  std::memcpy(buffer, sentence_.data(), length_);
  buffer[length_] = '\0';

  // Get message type (e.g., "$GNGGA", "#UNIHEADING")
  const char* msgType = GNSSParser::getMsgType(sentence_);

  if ((std::strncmp(msgType, "$GPGGA", 6) == 0 || std::strncmp(msgType, "$GNGGA", 6) == 0)) 
  {
    GNSSParser::GGA_DATA sGGA;
    GNSSParser::parseGGA(buffer, sGGA);
    if (sGGA.valid) 
    {
      _sCurrentData.latitude     = sGGA.latitude;
      _sCurrentData.longitude    = sGGA.longitude;
      _sCurrentData.fixQuality   = sGGA.fixQuality;
      _sCurrentData.satellites   = sGGA.satellitesUsed;
    }
  } 
  else if (std::strncmp(msgType, "#UNIHEADING", 11) == 0) 
  {
    GNSSParser::UNI_HEADING_DATA sHeading;
    GNSSParser::parseUniHeading(buffer, sHeading);
    if (sHeading.valid) 
    {
      updateHeadingFilter(sHeading.headingDeg);
    }
  }
}


void GNSSManager::updateHeadingFilter(float newHeadingDeg_) 
{
  _sCurrentData.headingDeg = _headingFilter.addValue(newHeadingDeg_);
}


float GNSSManager::getFilteredHeading(void) const 
{
  return _headingFilter.getAverage();
}


// namespace MyLib
// {

// struct GGAMSG
// {
//   float utc;
//   float lat;
//   float long;
// };


// template<size_t SIZE>
// getMsgType(std::array<char, SIZE> str)

// gga = getMsgType(const char* str, size_t len)
// if (gga)
// {
//   GGAMSG msg;
//   parseGGAMsg(string, msg);

//   _lattitude = msg.lat;
// }
// }


// void GNSSManager::parseMSG(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_) 
// {
//   if (length_ < 6 || (sentence_[0] != '$' && sentence_[0] != '#')) 
//   {
//     LOG_ERROR(Logger::Nodes::GNSS, "Le message reçu est tout cassé bozo: lenght: %d, sentence 0: %c", length_, sentence_)
//     return;
//   }

//   const char* tokens[32] = { nullptr };
//   size_t tokenCount = 0;
//   char* buffer = const_cast<char*>(sentence_.data()); // safe because we're working on a local copy

//   tokens[tokenCount++] = buffer;
//   for (size_t i = 0; i < length_ && tokenCount < 15; ++i) 
//   {
//     if (buffer[i] == ',') 
//     {
//       buffer[i] = '\0';
//       if (i + 1 < length_) 
//       {
//         tokens[tokenCount++] = &buffer[i + 1];
//       }
//     }
//   }

//   if ((strncmp(buffer, "$GPGGA", 6) == 0 || strncmp(buffer, "$GNGGA", 6) == 0) && tokenCount >= 8) 
//   {
//     currentData_.latitude = convertToDecimalDegrees(tokens[2], tokens[3][0]);
//     currentData_.longitude = convertToDecimalDegrees(tokens[4], tokens[5][0]);
//     currentData_.fixQuality = atoi(tokens[6]);
//     currentData_.satellites = atoi(tokens[7]);
//   }
//   if (strncmp(buffer, "#UNIHEADING", 6) == 0 && tokenCount >= 8) 
//   {
//     updateHeadingFilter(atof(tokens[12]));
//     currentData_.headingDeg = getFilteredHeading();
//   }
// }


// float GNSSManager::convertToDecimalDegrees(const char* nmeaCoord_, char direction_) 
// {
//   if (!nmeaCoord_ || strlen(nmeaCoord_) < 6)
//   {
//     LOG_ERROR(Logger::Nodes::GNSS, "Invalid NMEA Coord: %s", nmeaCoord_);
//     return 0.0f;
//   }

//   float degMin = atof(nmeaCoord_);
//   int degrees = static_cast<int>(ROUND(degMin / 100.0f));
//   float minutes = degMin - degrees * 100;
//   float decimal = degrees + minutes / 60.0f;

//   if (direction_ == 'S' || direction_ == 'W') 
//   {
//     decimal *= -1.0f;
//     return decimal;
//   }
//   else
//   {
//     LOG_ERROR(Logger::Nodes::GNSS, "Invalid direction: %c", direction_);
//     return 0.0f;
//   }
// }


// void GNSSManager::updateHeadingFilter(float newHeadingDeg) 
// {
//   float newRad = newHeadingDeg * DEG_TO_RAD;
//   float oldRad = headingBuffer_[headingIndex_];

//   // If buffer is not yet full, just grow it
//   if (headingCount_ < HEADING_FILTER_WINDOW) {
//       headingCount_++;
//   } else {
//       // Subtract oldest value from sum
//       sinSum_ -= sin(oldRad);
//       cosSum_ -= cos(oldRad);
//   }

//   // Replace value in ring buffer
//   headingBuffer_[headingIndex_] = newRad;
//   headingIndex_ = (headingIndex_ + 1) % HEADING_FILTER_WINDOW;

//   // Add new value
//   sinSum_ += sin(newRad);
//   cosSum_ += cos(newRad);

//   currentData_.headingDeg = getFilteredHeading();
// }


// float GNSSManager::getFilteredHeading(void) const 
// {
//   if (headingCount_ == 0) return 0.0;
//   float avgRad = atan2(sinSum_ / headingCount_, cosSum_ / headingCount_);
//   if (avgRad < 0) avgRad += 2 * PI;
//   return avgRad * RAD_TO_DEG;
// }


GNSS_DATA GNSSManager::getData(void) const 
{
  return _sCurrentData;
}


GNSSManager::~GNSSManager() 
{

}
