#include <utilities/IOManager.h>
#include <utilities/Settings.h>
#include <include/roboteam_ai/utilities/GameStateManager.hpp>
#include <include/roboteam_ai/utilities/Print.h>
#include "roboteam_proto/DemoRobot.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"

#include "interface/api/Input.h"
#include "utilities/Pause.h"
#include "world/Robot.h"

#include "roboteam_utils/normalize.h"

namespace rtt::ai::io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager io;

void IOManager::init(int teamId) {
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

    if (refData.blueteamonpositivehalf() ^ SETTINGS.isYellow()) {
        SETTINGS.setLeft(false);
    } else {
        SETTINGS.setLeft(true);
    }
    ai::GameStateManager::setRefereeData(refData);
}

void IOManager::handleFeedback(proto::RobotFeedback &feedback) {
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    feedbackMap.insert({feedback.id(), feedback});
}

const proto::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    auto msg = new proto::World();
    msg->CopyFrom(this->worldMsg);
    return *msg;
}

const proto::SSL_GeometryData &IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(geometryMutex);
    auto msg = new proto::SSL_GeometryData();
    msg->CopyFrom(this->geometryMsg);
    return *msg;
}

const proto::SSL_Referee &IOManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refereeMutex);
    auto msg = new proto::SSL_Referee();
    msg->CopyFrom(this->refDataMsg);
    return *msg;
}

void IOManager::publishRobotCommand(proto::RobotCommand cmd) {
    if (!pause->getPause()) {
        // the geneva cannot be received from world, so we set it when it gets sent.
        auto robot = world::world->getRobotForId(cmd.id(), true);
        if (robot) {
            if (cmd.geneva_state() == 3) {
                robot->setGenevaState(cmd.geneva_state());
            }

            /*
             *
             * if there is (recent) feedback we should not need to update internal state here
             * Otherwise we should. We need only do it when the new state is valid and different.
             */
            if (!robot->genevaStateIsDifferent(cmd.geneva_state()) || !robot->genevaStateIsValid(cmd.geneva_state())) {
                cmd.set_geneva_state(robot->getGenevaState());
            }

            //  if (!Constants::FEEDBACK_ENABLED() || !robot->hasRecentFeedback()) {
            robot->setGenevaState(cmd.geneva_state());
            //   }

            // only kick and chip when geneva is ready
            cmd.set_kicker(cmd.kicker() && robot->isGenevaReady());
            cmd.set_chipper(cmd.chipper() && robot->isGenevaReady());
            cmd.set_chip_kick_forced(cmd.chip_kick_forced() && robot->isGenevaReady());

            if (cmd.kicker()) {
                interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
            }

            if (cmd.chip_kick_forced()) {
                interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::DOTS, 36, 36, 8);
            }

            if (cmd.chipper()) {
                interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
            }

            robot->setDribblerState(cmd.dribbler());
        }
        // sometimes trees are terminated without having a role assigned.
        // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
        if (cmd.id() >= 0 && cmd.id() < 16) {
            robotCommandPublisher->send(cmd);
        }
    } else {
        //     ROS_ERROR("HALT!");
    }
}

const proto::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}


void IOManager::publishSettings(proto::Setting setting) {
    settingsPublisher->send(setting); 
}


}  // namespace rtt::ai::io
