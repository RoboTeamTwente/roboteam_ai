/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <roboteam_msgs/DemoRobot.h>
#include "../utilities/Pause.h"
#include "../world/Field.h"
#include "../world/Robot.h"
#include "IOManager.h"

namespace rtt {
namespace ai {
namespace io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;

IOManager::IOManager(bool subscribe, bool advertise) {
    if (subscribe) {
        // subscribe to all topics
        this->subscribeToWorldState();
        this->subscribeToGeometryData();
        this->subscribeToRobotFeedback();
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
    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
    this->robotFeedbackMsg = *robotfeedback;

    auto robot = world::world->getRobotForId(robotfeedback->id);

    if (robot) {
        robot->setWorkingGeneva(robotfeedback->genevaIsWorking);
        robot->setHasWorkingBallSensor(robotfeedback->ballSensorIsWorking);
        robot->setBatteryLow(robotfeedback->batteryLow);
    }

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

void IOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    // the geneva cannot be received from world, so we set it when it gets sent.
    auto robot = world::world->getRobotForId(cmd.id, true);
    if (robot) {

        // this is a failcheck; the geneva state will only be turned if it is possible (which is checked in robot)
        robot->setGenevaState(cmd.geneva_state);
        cmd.geneva_state = robot->getGenevaState();

        // only kick and chipp when geneva is ready
        cmd.kicker = cmd.kicker && robot->isGenevaReady();
        cmd.chipper = cmd.chipper && robot->isGenevaReady();
        cmd.kicker_forced = cmd.kicker_forced && robot->isGenevaReady();
        cmd.chipper_forced = cmd.chipper_forced && robot->isGenevaReady();

        robot->setDribblerState(cmd.dribbler);
    }
    // sometimes trees are terminated without having a role assigned.
    // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
    if (cmd.id >= 0 && cmd.id < 16) {
        robotCommandPublisher.publish(cmd);
    }

}

} // io
} // ai
} // rtt



