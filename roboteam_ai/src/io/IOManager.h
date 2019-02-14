#ifndef ROBOTEAM_AI_IO_MANAGER_H
#define ROBOTEAM_AI_IO_MANAGER_H

#include <iostream>
#include "ros/ros.h"
#include "roboteam_utils/constants.h"
#include <roboteam_msgs/GeometryData.h>
#include "roboteam_msgs/World.h"
#include <roboteam_msgs/RoleFeedback.h>
#include <roboteam_msgs/RobotCommand.h>
#include "roboteam_msgs/RefereeData.h"
#include <roboteam_msgs/DemoRobot.h>

namespace rtt {
namespace ai {
namespace io {

class IOManager {
private:

        ros::NodeHandle nodeHandle;

        roboteam_msgs::World world;
        roboteam_msgs::GeometryData geometry;
        roboteam_msgs::RoleFeedback roleFeedback;
        roboteam_msgs::RefereeData refData;
        roboteam_msgs::DemoRobot demoInfo;
        ros::Subscriber worldSubscriber;
        ros::Subscriber geometrySubscriber;
        ros::Subscriber roleFeedbackSubscriber;
        ros::Subscriber refereeSubscriber;
        ros::Subscriber demoInfoSubscriber;

        ros::Publisher robotCommandPublisher;
        void handleWorldState(const roboteam_msgs::WorldConstPtr &world);
        void handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry);
        void handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback);
        void handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData);
        void handleDemoInfo(const roboteam_msgs::DemoRobotPtr &demoInfo);

    public:
        explicit IOManager(bool subscribe = false, bool advertise = false);
        void subscribeToWorldState();
        void subscribeToGeometryData();
        void subscribeToRoleFeedback();
        void subscribeToRefereeData();
        void subscribeToDemoInfo();

        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);

        const roboteam_msgs::World &getWorldState();
        const roboteam_msgs::GeometryData &getGeometryData();
        const roboteam_msgs::RoleFeedback &getRoleFeedback();
        const roboteam_msgs::RefereeData &getRefereeData();
        const roboteam_msgs::DemoRobot &getDemoInfo();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H