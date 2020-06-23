#include <utilities/IOManager.h>
#include <utilities/Settings.h>

#include <include/roboteam_ai/utilities/GameStateManager.hpp>

#include "interface/api/Input.h"
#include "roboteam_utils/normalize.h"
#include "utilities/Pause.h"
#include "world_new/World.hpp"

namespace rtt::ai::io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager io;

IOManager::~IOManager() {
    delete worldSubscriber;
    delete geometrySubscriber;
    delete refSubscriber;
    delete feedbackSubscriber;
    delete robotCommandPublisher;
    delete settingsPublisher;
}

void IOManager::init(int teamId) {
    RTT_INFO("Setting up IO publishers/subscribers")
    worldSubscriber = new proto::Subscriber<proto::World>(proto::WORLD_CHANNEL, &IOManager::handleWorldState, this);
    geometrySubscriber = new proto::Subscriber<proto::SSL_GeometryData>(proto::GEOMETRY_CHANNEL, &IOManager::handleGeometry, this);
    refSubscriber = new proto::Subscriber<proto::SSL_Referee>(proto::REFEREE_CHANNEL, &IOManager::handleReferee, this);

    // set up advertisement to publish robotcommands and settings
    if (teamId == 1) {
        feedbackSubscriber = new proto::Subscriber<proto::RobotFeedback>(proto::FEEDBACK_SECONDARY_CHANNEL, &IOManager::handleFeedback, this);
        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        feedbackSubscriber = new proto::Subscriber<proto::RobotFeedback>(proto::FEEDBACK_PRIMARY_CHANNEL, &IOManager::handleFeedback, this);
        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_PRIMARY_CHANNEL);
    }
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleWorldState(proto::World &world) {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    this->worldMsg.CopyFrom(world);
}

void IOManager::handleGeometry(proto::SSL_GeometryData &geometryData) {
    std::lock_guard<std::mutex> lock(geometryMutex);
    this->geometryMsg = geometryData;
    hasReceivedGeom = true;
}

void IOManager::handleReferee(proto::SSL_Referee &refData) {
    std::lock_guard<std::mutex> lock(refereeMutex);
    this->refDataMsg = refData;
    roboteam_utils::rotate(&refData);

    // Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
    std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
    if (refData.yellow().name() == ROBOTEAM_TWENTE) {
        SETTINGS.setYellow(true);
    } else if (refData.blue().name() == ROBOTEAM_TWENTE) {
        SETTINGS.setYellow(false);
    }

    SETTINGS.setLeft(!(refData.blue_team_on_positive_half() ^ SETTINGS.isYellow()));
    ai::GameStateManager::setRefereeData(refData);
}

void IOManager::handleFeedback(proto::RobotFeedback &feedback) {
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    auto const result = feedbackMap.insert({feedback.id(), feedback});
    if (!result.second) { result.first->second = feedback; }
}

proto::World IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    return worldMsg;
}

proto::SSL_GeometryData IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(geometryMutex);
    return geometryMsg;
}

proto::SSL_Referee IOManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refereeMutex);
    return refDataMsg;
}

std::unordered_map<uint8_t, proto::RobotFeedback> IOManager::getFeedbackDataMap() {
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    return feedbackMap;
}

void IOManager::publishRobotCommand(proto::RobotCommand cmd) {
    if (!pause->getPause()) {
        auto const& [_, world] = world_new::World::instance();
        if (world->getWorld()) {
            // the geneva cannot be received from world, so we set it when it gets sent.
            auto robot = world->getWorld()->getRobotForId(cmd.id(), true);
            if (robot) {
                if (cmd.kicker()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::green, robot->get()->getId(), interface::Drawing::CIRCLES, 36, 36, 8);
                }
                if (cmd.chip_kick_forced()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::green, robot->get()->getId(), interface::Drawing::DOTS, 36, 36, 8);
                }
                if (cmd.chipper()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::yellow, robot->get()->getId(), interface::Drawing::CIRCLES, 36, 36, 8);
                }
                // robot->setDribblerState(cmd.dribbler());
            }
        }
        // sometimes trees are terminated without having a role assigned.
        // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
        if (cmd.id() >= 0 && cmd.id() < 16) {
            robotCommandPublisher->send(cmd);
        }
    }
}

const proto::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}

void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->send(setting); }
}  // namespace rtt::ai::io
