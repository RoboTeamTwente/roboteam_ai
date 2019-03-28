/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <roboteam_msgs/DemoRobot.h>
#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/utilities/Pause.h>
#include "IOManager.h"

namespace rtt {
namespace ai {
namespace io {

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
    this->worldMsg = *w;
}

void IOManager::handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry) {
    this->geometryMsg = *geometry;
}

void IOManager::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback) {
    this->roleFeedbackMsg = *rolefeedback;
}

void IOManager::handleDemoInfo(const roboteam_msgs::DemoRobotConstPtr &demoInfo) {
    this->demoInfoMsg = *demoInfo;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
    this->refDataMsg = *refData;
}

const roboteam_msgs::World &IOManager::getWorldState() {
    return this->worldMsg;
}

const roboteam_msgs::GeometryData &IOManager::getGeometryData() {
    return this->geometryMsg;
}

const roboteam_msgs::RoleFeedback &IOManager::getRoleFeedback() {
    return this->roleFeedbackMsg;
}

const roboteam_msgs::RefereeData &IOManager::getRefereeData() {
    return this->refDataMsg;
}

void IOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    if (! pause->getPause()) {
        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id)) {
            robotCommandPublisher.publish(cmd);
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



