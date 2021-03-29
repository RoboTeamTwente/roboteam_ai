//
// Created by rolf on 07-03-21.
//

#include "AISettings.h"
namespace rtt {
AISettings::AISettings(int initial_id) :
    id{initial_id},
    mode{GRSIM},
    robothub_send_ip{"127.0.0.1"}, //local host
    robothub_send_port{20011},//grsim default
    is_paused(true) {
  //these settings are for testing and convenience, basically
  if (initial_id == 1) {
    is_yellow = false;
    is_left = false;
  } else {
    is_yellow = true;
    is_left = true;
  }
}

int AISettings::getId() const {
  return id;
}
void AISettings::setId(int new_id) {
  id = new_id;
}
bool AISettings::isYellow() const {
  return is_yellow;
}
void AISettings::setYellow(bool isYellow) {
  is_yellow = isYellow;

}
bool AISettings::isLeft() const {
  return is_left;
}
void AISettings::setLeft(bool left) {
  is_left = left;
}
AISettings::CommunicationMode AISettings::getCommunicationMode() const {
  return mode;
}
void AISettings::setCommunicationMode(CommunicationMode new_mode) {
  mode = new_mode;
}
const std::string &AISettings::getRobothubSendIp() const {
  return robothub_send_ip;
}
void AISettings::setRobothubSendIp(const std::string &ip) {
  robothub_send_ip = ip;
}
int AISettings::getRobothubSendPort() const {
  return robothub_send_port;
}
void AISettings::setRobothubSendPort(int port) {
  robothub_send_port = port;
}
void AISettings::setPause(bool paused) {
  is_paused = paused; //TODO: when the AI is just paused, send a halt command to all robots
}
bool AISettings::isPaused() const {
  return is_paused;
}
}
