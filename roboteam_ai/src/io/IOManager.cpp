/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <roboteam_msgs/RobotCommand.h>
#include "IOManager.h"

namespace rtt {
namespace ai {
namespace io {

IOManager::IOManager(bool subscribe, bool advertise) {

    std::cout << "creating IOManager that can " << (subscribe ? "subscribe" : "t subscribe")
              << " and " << (advertise ? "advertise" : "'t advertise") << std::endl;

    if (subscribe) {
        // subscribe to all topics
        this->subscribeToWorldState();
        this->subscribeToGeometryData();
        this->subscribeToRoleFeedback();
        this->subscribeToRefereeData();
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
            this
    );
}

void IOManager::subscribeToGeometryData() {
    geometrySubscriber = nodeHandle.subscribe<roboteam_msgs::GeometryData>(
            rtt::TOPIC_GEOMETRY,
            1,
            &IOManager::handleGeometryData,
            this
    );
}

void IOManager::subscribeToRoleFeedback() {
    roleFeedbackSubscriber = nodeHandle.subscribe<roboteam_msgs::RoleFeedback>(
            rtt::TOPIC_ROLE_FEEDBACK,
            1,
            &IOManager::handleRobotFeedback,
            this
    );
}
void IOManager::subscribeToRefereeData() {
    //TODO: This constant TOPIC_REFEREE was not used consistently by the previous team, so if stuff goes wrong check if you are reading the correct topic.
    refereeSubscriber = nodeHandle.subscribe<roboteam_msgs::RefereeData>(
            rtt::TOPIC_REFEREE,
            1,
            &IOManager::handleRefereeData,
            this
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

const roboteam_msgs::World &IOManager::getWorldState() {
    return this->world;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
    this->refData = *refData;
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
    robotCommandPublisher.publish(cmd);
}

} // io
} // ai
} // rtt



