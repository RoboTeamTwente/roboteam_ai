#ifndef ROBOTEAM_AI_IO_MANAGER_H
#define ROBOTEAM_AI_IO_MANAGER_H

#include <iostream>
#include "ros/ros.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"
#include <roboteam_msgs/GeometryData.h>
#include "roboteam_msgs/World.h"
#include <roboteam_msgs/RoleFeedback.h>
#include <roboteam_msgs/RobotCommand.h>

namespace rtt {
namespace ai {
namespace io {

class IOManager {
    private:
        ros::NodeHandle nodeHandle;

        roboteam_msgs::World world;
        roboteam_msgs::GeometryData geometry;
        roboteam_msgs::RoleFeedback roleFeedback;
        ros::Subscriber worldSubscriber;
        ros::Subscriber geometrySubscriber;
        ros::Subscriber roleFeedbackSubscriber;
        ros::Publisher robotCommandPublisher;
        void handleWorldState(const roboteam_msgs::WorldConstPtr &world);
        void handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry);
        void handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback);

    public:
        IOManager();
        void subscribeToWorldState();
        void subscribeToGeometryData();
        void subscribeToRoleFeedback();

        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);

        const roboteam_msgs::World &getWorldState();
        const roboteam_msgs::GeometryData &getGeometryData();
        const roboteam_msgs::RoleFeedback &getRoleFeedback();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H