/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */


#include <roboteam_msgs/DemoRobot.h>
#include <roboteam_ai/src/demo/JoystickDemo.h>
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

void IOManager::handleWorldState(const roboteam_msgs::WorldConstPtr &world) {
    this->world = *world;
}

void IOManager::handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry) {
    this->geometry = *geometry;
}

void IOManager::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback) {
    this->roleFeedback = *rolefeedback;
}

void IOManager::handleDemoInfo(const roboteam_msgs::DemoRobotConstPtr &demoInfo) {
    this->demoInfo = *demoInfo;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
    this->refData = *refData;
}

const roboteam_msgs::World &IOManager::getWorldState() {
    return this->world;
}

const roboteam_msgs::GeometryData &IOManager::getGeometryData() {
    return this->geometry;
}

const roboteam_msgs::RoleFeedback &IOManager::getRoleFeedback() {
    return this->roleFeedback;
}

const roboteam_msgs::RefereeData &IOManager::getRefereeData() {
    return this->refData;
}

void IOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    if (demo::JoystickDemo::checkIfDemoSafe(cmd.id)) {
        robotCommandPublisher.publish(cmd);
    }
    else {
        ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(cmd.id).c_str());
    }
}
const roboteam_msgs::DemoRobot &IOManager::getDemoInfo() {
    return this->demoInfo;
}


} // io
} // ai
} // rtt



