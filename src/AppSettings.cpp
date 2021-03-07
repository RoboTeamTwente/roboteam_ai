//
// Created by rolf on 07-03-21.
//

#include "AppSettings.h"
const std::string &AppSettings::getRefereeIp() const {
  return referee_ip;
}
void AppSettings::setRefereeIp(const std::string &refereeIp) {
  referee_ip = refereeIp;
}
int AppSettings::getRefereePort() const {
  return referee_port;
}
void AppSettings::setRefereePort(int port) {
  referee_port = port;
}
const std::string &AppSettings::getVisionIp() const {
  return vision_ip;
}
void AppSettings::setVisionIp(const std::string &visionIp) {
  vision_ip = visionIp;
}
int AppSettings::getVisionPort() const {
  return vision_port;
}
void AppSettings::setVisionPort(int port) {
  vision_port = port;
}


