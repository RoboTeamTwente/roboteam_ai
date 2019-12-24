#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include <mutex>

#include "roboteam_world/world/settings.hpp"

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

namespace rtt::ai {
class Pause;

namespace io {

class IOManager {
private:
        proto::World worldMsg;
        proto::SSL_GeometryData geometryMsg;
        proto::RobotFeedback robotFeedbackMsg;
        proto::SSL_Referee refDataMsg;
        proto::DemoRobot demoInfoMsg;
        ::rtt::world::settings::Settings* settings{};

        proto::Subscriber<proto::World> * worldSubscriber{};
        void handleWorldState(proto::World & world);

        proto::Subscriber<proto::SSL_GeometryData> * geometrySubscriber{};
        void handleGeometry(proto::SSL_GeometryData & geometryData);

        proto::Subscriber<proto::SSL_Referee> * refSubscriber{};
        void handleReferee(proto::SSL_Referee & refData);

        proto::Subscriber<proto::RobotFeedback> * feedbackSubscriber{};
        void handleFeedback(proto::RobotFeedback & feedback);

        proto::Publisher<proto::RobotCommand> * robotCommandPublisher{};
        proto::Publisher<proto::Setting> * settingsPublisher{};

      rtt::ai::Pause* pause{};

public:
        IOManager() = default;
        /**
         * Constructs an IOManager from a settings object
         * @param settings Parameter MUST outlive the duration of the struct, otherwise a dangling pointer will be used in future interaction
         */
        explicit IOManager(::rtt::world::settings::Settings& settings);
        void publishRobotCommand(proto::RobotCommand cmd);
        void publishSettings();
        void init();
        const proto::World &getWorldState();
        const proto::SSL_GeometryData &getGeometryData();
        const proto::RobotFeedback &getRobotFeedback();
        const proto::SSL_Referee &getRefereeData();
        const proto::DemoRobot &getDemoInfo();

        static std::mutex worldStateMutex;
        static std::mutex geometryMutex;
        static std::mutex robotFeedbackMutex;
        static std::mutex refereeMutex;
        static std::mutex demoMutex;

        bool hasReceivedGeom = false;
};

extern IOManager* io;

} // io
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H