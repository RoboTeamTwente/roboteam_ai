#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include "ros/ros.h"
#include "roboteam_utils/constants.h"
#include <roboteam_msgs/GeometryData.h>
#include "roboteam_msgs/World.h"
#include <roboteam_msgs/RobotFeedback.h>
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
        roboteam_msgs::RobotFeedback robotFeedbackMsg;

        ros::Subscriber worldSubscriber;
        ros::Subscriber geometrySubscriber;
        ros::Subscriber roleFeedbackSubscriber;


        ros::Publisher robotCommandPublisher;
        void handleWorldState(const roboteam_msgs::WorldConstPtr &world);
        void handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry);
        void handleRobotFeedback(const roboteam_msgs::RobotFeedbackConstPtr &robotfeedback);

    public:
        explicit IOManager(bool subscribe = false, bool advertise = false);
        void subscribeToWorldState();
        void subscribeToGeometryData();
        void subscribeToRobotFeedback();


        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);

        const roboteam_msgs::World &getWorldState();
        const roboteam_msgs::GeometryData &getGeometryData();
        const roboteam_msgs::RobotFeedback &getRobotFeedback();

        static std::mutex worldStateMutex;
        static std::mutex geometryMutex;
        static std::mutex robotFeedbackMutex;


};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H