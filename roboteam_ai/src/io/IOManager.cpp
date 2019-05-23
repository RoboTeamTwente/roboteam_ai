/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <roboteam_msgs/DemoRobot.h>
#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/utilities/Pause.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include "IOManager.h"


namespace rtt {
namespace ai {
namespace io {
    std::mutex IOManager::mutex;

IOManager::IOManager(bool subscribe, bool advertise) {
    if (subscribe) {
        // subscribe to all topics
        this->subscribeToWorldState();
        this->subscribeToGeometryData();
        this->subscribeToRoleFeedback();
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

void IOManager::subscribeToRoleFeedback() {
    roleFeedbackSubscriber = nodeHandle.subscribe<roboteam_msgs::RoleFeedback>(
            rtt::TOPIC_ROLE_FEEDBACK,
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
    std::lock_guard<std::mutex> lock(mutex);
    this->worldMsg = *w;
    world::world->updateWorld(this->worldMsg);
}

void IOManager::handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry) {
    std::lock_guard<std::mutex> lock(mutex);
    this->geometryMsg = *geometry;
    world::field->set_field(this->geometryMsg.field);
}

void IOManager::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback) {
    std::lock_guard<std::mutex> lock(mutex);
    this->roleFeedbackMsg = *rolefeedback;
}

void IOManager::handleDemoInfo(const roboteam_msgs::DemoRobotConstPtr &demoInfo) {
    std::lock_guard<std::mutex> lock(mutex);
    this->demoInfoMsg = *demoInfo;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
    std::lock_guard<std::mutex> lock(mutex);
    this->refDataMsg = *refData;
    GameStateManager::setRefereeData(this->refDataMsg);
}

const roboteam_msgs::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(mutex);
    return this->worldMsg;
}

const roboteam_msgs::GeometryData &IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(mutex);
    return this->geometryMsg;
}

const roboteam_msgs::RoleFeedback &IOManager::getRoleFeedback() {
    std::lock_guard<std::mutex> lock(mutex);
    return this->roleFeedbackMsg;
}

const roboteam_msgs::RefereeData &IOManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(mutex);
    return this->refDataMsg;
}

void IOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    if (! pause->getPause()) {
        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id)) {

            // the geneva cannot be received from world, so we set it when it gets sent.
            auto robot = world::world->getRobotForId(cmd.id, true);
            if (robot) {

                robot->setGenevaState(cmd.geneva_state);
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
    return this->demoInfoMsg;
}


} // io
} // ai
} // rtt



