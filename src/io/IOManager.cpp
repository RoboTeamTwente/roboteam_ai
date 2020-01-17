/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <Settings/Settings.h>
#include <include/roboteam_ai/interface/api/Output.h>
#include <io/IOManager.h>
#include "roboteam_proto/DemoRobot.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"

#include "demo/JoystickDemo.h"
#include "interface/api/Input.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "world/Field.h"
#include "world/Robot.h"

#include "roboteam_utils/normalize.h"

namespace rtt::ai::io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager io;

void IOManager::handleWorldState(proto::World &world) {
    std::lock_guard<std::mutex> lock(worldStateMutex);

    if (!SETTINGS.isLeft()) {
        std::cout << "rotating message" << std::endl;
        roboteam_utils::rotate(&world);
    }

    this->worldMsg = world;
    world::world->updateWorld(this->worldMsg);
}

void IOManager::handleGeometry(proto::SSL_GeometryData &sslData) {
    std::lock_guard<std::mutex> lock(geometryMutex);

    // protobuf objects are not very long-lasting so convert it into an object which we can store way longer in field
    FieldMessage msg = FieldMessage(sslData.field());
    this->geometryMsg = sslData;

    world::field->set_field(msg);
    hasReceivedGeom = true;
}

void IOManager::handleReferee(proto::SSL_Referee &refData) {
    std::lock_guard<std::mutex> lock(refereeMutex);

    if (interface::Output::usesRefereeCommands()) {
        // Rotate the data from the referee (designated position, e.g. for ballplacement)
        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&refData);
        }

        this->refDataMsg = refData;

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

        GameStateManager::setRefereeData(refData);
    }
}

const proto::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    return this->worldMsg;
}

const proto::SSL_GeometryData &IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(geometryMutex);
    return this->geometryMsg;
}

const proto::RobotFeedback &IOManager::getRobotFeedback() {
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    return this->robotFeedbackMsg;
}

const proto::SSL_Referee &IOManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refereeMutex);
    return this->refDataMsg;
}

void IOManager::publishRobotCommand(proto::RobotCommand cmd) {
    if (!pause->getPause()) {
        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id())) {
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
            //   ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(cmd.id).c_str());
        }
    } else {
        //     ROS_ERROR("HALT!");
    }
}

const proto::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}

void IOManager::init() {
    worldSubscriber = new proto::Subscriber<proto::World>(proto::WORLD_CHANNEL, &IOManager::handleWorldState, this);
    geometrySubscriber = new proto::Subscriber<proto::SSL_GeometryData>(proto::GEOMETRY_CHANNEL, &IOManager::handleGeometry, this);
    refSubscriber = new proto::Subscriber<proto::SSL_Referee>(proto::REFEREE_CHANNEL, &IOManager::handleReferee, this);

    // set up advertisement to publish robotcommands and settings
    if (SETTINGS.getId() == 1) {
        feedbackSubscriber = new proto::Subscriber<proto::RobotFeedback>(proto::FEEDBACK_SECONDARY_CHANNEL, &IOManager::handleFeedback, this);

        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        feedbackSubscriber = new proto::Subscriber<proto::RobotFeedback>(proto::FEEDBACK_PRIMARY_CHANNEL, &IOManager::handleFeedback, this);

        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_PRIMARY_CHANNEL);
    }
}

void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->send(setting); }

void IOManager::handleFeedback(proto::RobotFeedback &feedback) {
    if (Constants::FEEDBACK_ENABLED()) {
        std::lock_guard<std::mutex> lock(robotFeedbackMutex);
        this->robotFeedbackMsg = feedback;

        auto robot = world::world->getRobotForId(feedback.id());

        if (robot) {
            // indicate that now is last time the robot has received feedback
            robot->UpdateFeedbackReceivedTime();

            // override properties:
            robot->setWorkingGeneva(feedback.genevaisworking());
            robot->setHasWorkingBallSensor(feedback.ballsensorisworking());
            robot->setBatteryLow(feedback.batterylow());
            // robot->setGenevaStateFromFeedback(feedback.genevastate());
        }
    }
}

}  // namespace rtt::ai::io
