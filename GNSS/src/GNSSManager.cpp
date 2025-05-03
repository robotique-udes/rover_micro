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
  while (GNSSSerial->available()) {
    char c = GNSSSerial->read();
    if (c == '\n') {
        parseNMEA(buffer_);
        buffer_ = "";
    } else if (c != '\r') {
        buffer_ += c;
    }
  }
}


void GNSSManager::parseNMEA(const String& sentence_) {
  if (sentence_.startsWith("$GPGGA")) {
      int index = 0;
      String parts[15];
      for (size_t i = 0; i < sentence_.length() && index < 15; ++i) {
          if (sentence_[i] == ',') {
              ++index;
          } else {
              parts[index] += sentence_[i];
          }
      }

      if (index >= 7) {
          currentData_.latitude = convertToDecimalDegrees(parts[2], parts[3].charAt(0));
          currentData_.longitude = convertToDecimalDegrees(parts[4], parts[5].charAt(0));
          currentData_.validFix = parts[6].toInt() > 0;
          currentData_.satellites = parts[7].toInt();
      }
  }
}


double GNSSManager::convertToDecimalDegrees(const String& nmeaCoord_, char direction_) {
  if (nmeaCoord_.length() < 6) return 0.0;
  double degMin = nmeaCoord_.toDouble();
  int degrees = static_cast<int>(degMin / 100);
  double minutes = degMin - degrees * 100;
  double decimal = degrees + minutes / 60.0;
  if (direction_ == 'S' || direction_ == 'W') decimal *= -1;
  return decimal;
}


GNSSData GNSSManager::getData() const {
  return currentData_;
}


GNSSManager::~GNSSManager() 
{

}
