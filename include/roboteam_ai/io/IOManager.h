#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include "constants.h"
#include "World.pb.h"
#include <RobotFeedback.pb.h>
#include <RobotCommand.pb.h>
#include "Referee.pb.h"
#include <DemoRobot.pb.h>
#include <mutex>
#include <Subscriber.h>
#include <Publisher.h>

#include <messages_robocup_ssl_geometry.pb.h>
#include <GeometryData.pb.h>
#include <include/roboteam_ai/utilities/FieldMessage.h>
#include <messages_robocup_ssl_referee.pb.h>

namespace rtt {
namespace ai {
class Pause;

namespace io {

class IOManager {
private:

// Map that converts SSL line and arc names to the more clear RoboTeam ones.


        roboteam_proto::World worldMsg;
        roboteam_proto::SSL_GeometryData geometryMsg;
        roboteam_proto::RobotFeedback robotFeedbackMsg;
        roboteam_proto::SSL_Referee refDataMsg;
        roboteam_proto::DemoRobot demoInfoMsg;

        roboteam_proto::Subscriber * worldSubscriber;
        void handleWorldState(roboteam_proto::World & world);

        roboteam_proto::Subscriber * geometrySubscriber;
        void handleGeometry(roboteam_proto::SSL_GeometryData & geometryData);

        roboteam_proto::Subscriber * refSubscriber;
        void handleReferee(roboteam_proto::SSL_Referee & refData);

  roboteam_proto::Publisher * robotCommandPublisher;

  //        ros::Subscriber worldSubscriber;
//        ros::Subscriber geometrySubscriber;
//        ros::Subscriber roleFeedbackSubscriber;
//        ros::Subscriber refereeSubscriber;
//        ros::Subscriber demoInfoSubscriber;
//
//        ros::Publisher robotCommandPublisher;
        rtt::ai::Pause* pause;


    public:
        explicit IOManager() = default;
        void publishRobotCommand(roboteam_proto::RobotCommand cmd);
        void init();
        const roboteam_proto::World &getWorldState();
        const roboteam_proto::SSL_GeometryData &getGeometryData();
        const roboteam_proto::RobotFeedback &getRobotFeedback();
        const roboteam_proto::SSL_Referee &getRefereeData();
        const roboteam_proto::DemoRobot &getDemoInfo();

        static std::mutex worldStateMutex;
        static std::mutex geometryMutex;
        static std::mutex robotFeedbackMutex;
        static std::mutex refereeMutex;
        static std::mutex demoMutex;

};

extern IOManager io;

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H