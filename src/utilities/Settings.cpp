//
// Created by mrlukasbos on 1-9-19.
//

#include "utilities/Settings.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

namespace rtt {
Settings SETTINGS;

constexpr int PRIMARY_AI_ID = 0;

Settings::Settings() {}

void Settings::init(int id) { setId(id); }

void Settings::handleSettingsFromPrimaryAI(bool otherIsYellow, bool otherIsLeft, RobotHubMode otherMode, std::string otherVisionIp, int otherVisionPort, std::string otherRefereeIp,
                                           int otherRefereePort, std::string otherRobotHubSendIp, int otherRobotHubSendPort) {
    this->setYellow(!otherIsYellow);
    this->left = !otherIsLeft;
    this->robotHubMode = otherMode;
    this->visionIp = otherVisionIp;
    this->visionPort = visionPort;
    this->refereeIp = otherRefereeIp;
    this->refereePort = otherRefereePort;
    this->robothubSendIp = otherRobotHubSendIp;
    this->robothubSendPort = otherRobotHubSendPort;
}

int Settings::getId() const { return id; }
bool Settings::isPrimaryAI() const { return this->id == PRIMARY_AI_ID; }

void Settings::setId(int id) { Settings::id = id; }

bool Settings::isYellow() const { return yellow; }

bool Settings::setYellow(bool yellow) {
    bool hasWantedColor = false;

    if (ai::io::io.obtainTeamColorChannel(yellow)) {
        // We could obtain the necessary channel
        this->yellow = yellow;
        hasWantedColor = true;
    }

    return hasWantedColor;
}

bool Settings::isLeft() const { return left; }

void Settings::setLeft(bool _left) {
    if (this->isPrimaryAI()) this->left = _left;
}

Settings::RobotHubMode Settings::getRobotHubMode() const { return this->robotHubMode; }

bool Settings::setRobotHubMode(RobotHubMode mode) {
    bool changedMode = false;

    // We can only switch mode if we are the primary AI
    if (this->isPrimaryAI()) {
        this->robotHubMode = mode;
        changedMode = true;
    } else {
        RTT_INFO("This secondary AI can not alter settings")
    }

    return changedMode;
}

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

std::string Settings::robotHubModeToString(RobotHubMode mode) {
    switch (mode) {
        case RobotHubMode::BASESTATION:
            return "Basestation";
        case RobotHubMode::SIMULATOR:
            return "Simulator";
        default:
            return "Unknown";
    }
}

}  // namespace rtt