//
// Created by mrlukasbos on 1-9-19.
//

#include "utilities/Settings.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

namespace rtt {
Settings SETTINGS;

constexpr int PRIMARY_AI_ID = 0;

void Settings::init(int id) { setId(id); }

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
bool Settings::isPrimaryAI() const { return this->id == PRIMARY_AI_ID; }

void Settings::setId(int id) { Settings::id = id; }

bool Settings::isYellow() const { return yellow; }

bool Settings::setYellow(bool yellow) {
    bool switched = rtt::ai::io::io.switchTeamColorChannel(yellow);

    if (switched) {
        Settings::yellow = yellow;
        RTT_INFO(yellow ? "Team color changed: Yellow" : "Team color changed: Blue")
    } else {
        RTT_ERROR("Failed to open channel. Is another AI already running on commands channel ", yellow ? "YELLOW" : "BLUE", "?")
    }

    return switched;
}

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