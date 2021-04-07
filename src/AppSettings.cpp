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
proto::ObserverSettings AppSettings::obsMessage() const{

  proto::ObserverSettings settings;
  settings.set_visionip(vision_ip);
  settings.set_visionport(vision_port);
  settings.set_refereeip(referee_ip);
  settings.set_refereeport(referee_port);
  return settings;
}
proto::Setting AppSettings::toMessage() const {
  proto::Setting setting;
  setting.mutable_obs_settings()->CopyFrom(obsMessage());
  setting.mutable_rh_settings()->CopyFrom(rhMessage());
  return setting;
}

proto::RobotHubSettings AppSettings::rhMessage() const {
  proto::RobotHubSettings settings;
  settings.set_robothubip(robothub_ip);
  settings.set_robothubport(robothub_port);
  settings.set_serialmode(mode == SerialMode::SERIAL);//TODO: extend proto with an enum to support more modes
  return settings;
}


