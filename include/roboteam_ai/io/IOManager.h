#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include <mutex>

#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include "roboteam_proto/Referee.pb.h"
#include "roboteam_proto/Setting.pb.h"
#include "roboteam_proto/DemoRobot.pb.h"
#include "roboteam_proto/Subscriber.h"
#include "roboteam_proto/Publisher.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include "roboteam_proto/GeometryData.pb.h"
#include "roboteam_proto/messages_robocup_ssl_referee.pb.h"

#include "include/roboteam_ai/world/FieldMessage.h"
#include "utilities/Constants.h"

namespace rtt {
namespace ai {
class Pause;

namespace io {

class IOManager {
private:
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

        roboteam_proto::Subscriber * feedbackSubscriber;
        void handleFeedback(roboteam_proto::RobotFeedback & feedback);

        roboteam_proto::Publisher * publisher;
        rtt::ai::Pause* pause;

public:
        explicit IOManager() = default;
        void publishRobotCommand(roboteam_proto::RobotCommand cmd);
        void publishSettings(roboteam_proto::Setting setting);
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

        bool hasReceivedGeom = false;
        bool hasReceivedWorld = false;
};

extern IOManager io;

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H