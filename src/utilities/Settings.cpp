//
// Created by mrlukasbos on 1-9-19.
//

#include "utilities/Settings.h"

namespace rtt {
Settings SETTINGS;

void Settings::init(int id) {
    setId(id);
}

proto::Setting Settings::toMessage() {
    proto::Setting setting;
    setting.set_id(id);
    setting.set_isleft(left);
    setting.set_isyellow(yellow);
    setting.set_serialmode(serialMode);
    setting.set_refereeip(refereeIp);
    setting.set_refereeport(refereePort);
    setting.set_visionip(visionIp);
    setting.set_visionport(visionPort);
    setting.set_robothubsendip(robothubSendIp);
    setting.set_robothubsendport(robothubSendPort);
    return setting;
}

int Settings::getId() const { return id; }

void Settings::setId(int id) { Settings::id = id; }

bool Settings::isYellow() const { return yellow; }

void Settings::setYellow(bool yellow) { Settings::yellow = yellow; }

bool Settings::isLeft() const { return left; }

void Settings::setLeft(bool left) { Settings::left = left; }

bool Settings::isSerialMode() const { return serialMode; }

void Settings::setSerialMode(bool serialMode) { Settings::serialMode = serialMode; }

const std::string &Settings::getVisionIp() const { return visionIp; }

void Settings::setVisionIp(const std::string &visionIp) { Settings::visionIp = visionIp; }

int Settings::getVisionPort() const { return visionPort; }

void Settings::setVisionPort(int visionPort) { Settings::visionPort = visionPort; }

const std::string &Settings::getRefereeIp() const { return refereeIp; }

void Settings::setRefereeIp(const std::string &refereeIp) { Settings::refereeIp = refereeIp; }

int Settings::getRefereePort() const { return refereePort; }

void Settings::setRefereePort(int refereePort) { Settings::refereePort = refereePort; }

const std::string &Settings::getRobothubSendIp() const { return robothubSendIp; }

void Settings::setRobothubSendIp(const std::string &robothubSendIp) { Settings::robothubSendIp = robothubSendIp; }

int Settings::getRobothubSendPort() const { return robothubSendPort; }

void Settings::setRobothubSendPort(int robothubSendPort) { Settings::robothubSendPort = robothubSendPort; }
}  // namespace rtt