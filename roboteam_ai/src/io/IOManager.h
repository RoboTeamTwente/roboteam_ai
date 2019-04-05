#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include "ros/ros.h"
#include "roboteam_utils/constants.h"
#include <roboteam_msgs/GeometryData.h>
#include "roboteam_msgs/World.h"
#include <roboteam_msgs/RoleFeedback.h>
#include <roboteam_msgs/RobotCommand.h>
#include "roboteam_msgs/RefereeData.h"
#include <roboteam_msgs/DemoRobot.h>
#include <mutex>

namespace rtt {
namespace ai {
class Pause;

namespace io {

class IOManager {
private:

        ros::NodeHandle nodeHandle;

        roboteam_msgs::World worldMsg;
        roboteam_msgs::GeometryData geometryMsg;
        roboteam_msgs::RoleFeedback roleFeedbackMsg;
        roboteam_msgs::RefereeData refDataMsg;
        roboteam_msgs::DemoRobot demoInfoMsg;
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
        void handleDemoInfo(const roboteam_msgs::DemoRobotConstPtr &demoInfo);
        rtt::ai::Pause* pause;

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

        static std::mutex mutex;
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H