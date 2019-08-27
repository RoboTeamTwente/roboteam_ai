/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <roboteam_msgs/DemoRobot.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "include/roboteam_ai/demo/JoystickDemo.h"
#include "include/roboteam_ai/utilities/Pause.h"
#include "include/roboteam_ai/world/Field.h"
#include "include/roboteam_ai/world/Robot.h"
#include "include/roboteam_ai/utilities/GameStateManager.hpp"
#include "include/roboteam_ai/io/IOManager.h"


namespace rtt {
namespace ai {
namespace io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager::IOManager(bool subscribe, bool advertise) {
    while (!nodeHandle.ok()) {}
    if (subscribe) {
        // subscribe to all topics
        this->subscribeToWorldState();
        this->subscribeToGeometryData();
        this->subscribeToRobotFeedback();
        this->subscribeToRefereeData();
        this->subscribeToDemoInfo();
    }

    if (advertise) {
        // set up advertisement to publish robotcommands
        robotCommandPublisher = nodeHandle.advertise<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 100);
    }
}

void IOManager::subscribeToWorldState() {
    worldSubscriber = nodeHandle.subscribe<roboteam_msgs::World>(
            rtt::TOPIC_WORLD_STATE,
            1,
            &IOManager::handleWorldState,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}

void IOManager::subscribeToGeometryData() {

    geometrySubscriber = nodeHandle.subscribe<roboteam_msgs::GeometryData>(
            rtt::TOPIC_GEOMETRY,
            100,
            &IOManager::handleGeometryData,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}

void IOManager::subscribeToRobotFeedback() {
    roleFeedbackSubscriber = nodeHandle.subscribe<roboteam_msgs::RobotFeedback>(
            "robot_feedback",
            100,
            &IOManager::handleRobotFeedback,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}

void IOManager::subscribeToRefereeData() {
    //TODO: This constant TOPIC_REFEREE was not used consistently by the previous team, so if stuff goes wrong check if you are reading the correct topic.
    refereeSubscriber = nodeHandle.subscribe<roboteam_msgs::RefereeData>(
            "vision_refbox", //vision_referee or vision_refbox
            100,
            &IOManager::handleRefereeData,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}

void IOManager::subscribeToDemoInfo() {
    demoInfoSubscriber = nodeHandle.subscribe<roboteam_msgs::DemoRobot>(
            "demo_info",
            100,
            &IOManager::handleDemoInfo,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}

void IOManager::handleWorldState(const roboteam_msgs::WorldConstPtr &w) {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    this->worldMsg = *w;
    world::world->updateWorld(this->worldMsg);
}

void IOManager::handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry) {
    std::lock_guard<std::mutex> lock(geometryMutex);
    this->geometryMsg = *geometry;
    world::field->set_field(this->geometryMsg.field);
}

void IOManager::handleRobotFeedback(const roboteam_msgs::RobotFeedbackConstPtr &robotfeedback) {
    if (Constants::FEEDBACK_ENABLED()) {
        std::lock_guard<std::mutex> lock(robotFeedbackMutex);
        this->robotFeedbackMsg = *robotfeedback;

        auto robot = world::world->getRobotForId(robotfeedback->id);

        if (robot) {

            // indicate that now is last time the robot has received feedback
            robot->UpdateFeedbackReceivedTime();

            // override properties:
            robot->setWorkingGeneva(robotfeedback->genevaIsWorking);
            robot->setHasWorkingBallSensor(robotfeedback->ballSensorIsWorking);
            robot->setBatteryLow(robotfeedback->batteryLow);
            //robot->setGenevaStateFromFeedback(robotfeedback->genevaState);
        }
    }
}

void IOManager::handleDemoInfo(const roboteam_msgs::DemoRobotConstPtr &demoInfo) {
    std::lock_guard<std::mutex> lock(demoMutex);
    this->demoInfoMsg = *demoInfo;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
    std::lock_guard<std::mutex> lock(refereeMutex);
    this->refDataMsg = *refData;
    GameStateManager::setRefereeData(this->refDataMsg);
}

const roboteam_msgs::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    return this->worldMsg;
}

const roboteam_msgs::GeometryData &IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(geometryMutex);
    return this->geometryMsg;
}

const roboteam_msgs::RobotFeedback &IOManager::getRobotFeedback() {
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    return this->robotFeedbackMsg;
}

const roboteam_msgs::RefereeData &IOManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refereeMutex);
    return this->refDataMsg;
}

void IOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    if (! pause->getPause()) {
        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id)) {

            // the geneva cannot be received from world, so we set it when it gets sent.
            auto robot = world::world->getRobotForId(cmd.id, true);
            if (robot) {
                if (cmd.geneva_state == 3) {
                    robot->setGenevaState(cmd.geneva_state);
                }
                /*
                 *
                 * if there is (recent) feedback we should not need to update internal state here
                 * Otherwise we should. We need only do it when the new state is valid and different.
                 */
                if (!robot->genevaStateIsDifferent(cmd.geneva_state) || !robot->genevaStateIsValid(cmd.geneva_state)) {
                    cmd.geneva_state = robot->getGenevaState();
                }

              //  if (!Constants::FEEDBACK_ENABLED() || !robot->hasRecentFeedback()) {
                    robot->setGenevaState(cmd.geneva_state);
             //   }

                // only kick and chip when geneva is ready
                cmd.kicker = cmd.kicker && robot->isGenevaReady();
                cmd.chipper = cmd.chipper && robot->isGenevaReady();
                cmd.kicker_forced = cmd.kicker_forced && robot->isGenevaReady();
                cmd.chipper_forced = cmd.chipper_forced && robot->isGenevaReady();

                if (cmd.kicker) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
                }

                if (cmd.kicker_forced) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::DOTS, 36, 36, 8);
                }


                if (cmd.chipper) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
                }

                if (cmd.chipper_forced) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::DOTS, 36, 36, 8);
                }

                robot->setDribblerState(cmd.dribbler);
            }
            // sometimes trees are terminated without having a role assigned.
            // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
            if (cmd.id >= 0 && cmd.id < 16) {
                robotCommandPublisher.publish(cmd);
            }
        }
        else {
            ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(cmd.id).c_str());
        }
    }
    else {
        ROS_ERROR("HALT!");
    }
}

const roboteam_msgs::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}


} // io
} // ai
} // rtt



