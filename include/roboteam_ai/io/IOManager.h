#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include "constants.h"
#include <GeometryData.pb.h>
#include "World.pb.h"
#include <RobotFeedback.pb.h>
#include <RobotCommand.pb.h>
#include "Referee.pb.h"
#include <DemoRobot.pb.h>
#include <mutex>

namespace rtt {
namespace ai {
class Pause;

namespace io {

class IOManager {
private:

//        ros::NodeHandle nodeHandle;

        roboteam_proto::World worldMsg;
        roboteam_proto::GeometryData geometryMsg;
        roboteam_proto::RobotFeedback robotFeedbackMsg;
        roboteam_proto::RefereeData refDataMsg;
        roboteam_proto::DemoRobot demoInfoMsg;
//        ros::Subscriber worldSubscriber;
//        ros::Subscriber geometrySubscriber;
//        ros::Subscriber roleFeedbackSubscriber;
//        ros::Subscriber refereeSubscriber;
//        ros::Subscriber demoInfoSubscriber;
//
//        ros::Publisher robotCommandPublisher;
        rtt::ai::Pause* pause;

    public:
        explicit IOManager(bool subscribe = false, bool advertise = false);
        void subscribeToWorldState();
        void subscribeToGeometryData();
        void subscribeToRobotFeedback();
        void subscribeToRefereeData();
        void subscribeToDemoInfo();

        void publishRobotCommand(roboteam_proto::RobotCommand cmd);

        const roboteam_proto::World &getWorldState();
        const roboteam_proto::GeometryData &getGeometryData();
        const roboteam_proto::RobotFeedback &getRobotFeedback();
        const roboteam_proto::RefereeData &getRefereeData();
        const roboteam_proto::DemoRobot &getDemoInfo();

        static std::mutex worldStateMutex;
        static std::mutex geometryMutex;
        static std::mutex robotFeedbackMutex;
        static std::mutex refereeMutex;
        static std::mutex demoMutex;

};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H