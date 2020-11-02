#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include <mutex>

#include <roboteam_proto/RobotCommand.pb.h>
#include <roboteam_proto/RobotFeedback.pb.h>
#include <roboteam_proto/Setting.pb.h>
#include <roboteam_proto/World.pb.h>
#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include <Subscriber.h>
#include <Publisher.h>

#include "include/roboteam_ai/world/Field.h"
#include "utilities/Constants.h"

namespace rtt::world {
class World;
}

namespace rtt::ai {
class Pause;


namespace io {
using namespace rtt::world;

class IOManager {
   private:
    proto::World worldMsg;
    proto::SSL_GeometryData geometryMsg;
    proto::RobotFeedback robotFeedbackMsg;
    proto::SSL_Referee refDataMsg;

    std::unordered_map<uint8_t, proto::RobotFeedback> feedbackMap;

    proto::Subscriber<proto::World> *worldSubscriber;
    void handleWorldState(proto::World &world);

    proto::Subscriber<proto::SSL_GeometryData> *geometrySubscriber;
    void handleGeometry(proto::SSL_GeometryData &geometryData);

    proto::Subscriber<proto::SSL_Referee> *refSubscriber;
    void handleReferee(proto::SSL_Referee &refData);

    proto::Subscriber<proto::RobotFeedback> *feedbackSubscriber;
    void handleFeedback(proto::RobotFeedback &feedback);

    proto::Publisher<proto::RobotCommand> *robotCommandPublisher;
    proto::Publisher<proto::Setting> *settingsPublisher;

    rtt::ai::Pause *pause;

   public:
    ~IOManager();
    explicit IOManager() = default;
    void publishRobotCommand(proto::RobotCommand cmd, rtt::world::World const* world);
    void publishSettings(proto::Setting setting);
    void init(int teamId);
    proto::World getWorldState();
    proto::SSL_GeometryData getGeometryData();
    std::unordered_map<uint8_t, proto::RobotFeedback> getFeedbackDataMap();

    proto::SSL_Referee getRefereeData();

    static std::mutex worldStateMutex;
    static std::mutex geometryMutex;
    static std::mutex robotFeedbackMutex;
    static std::mutex refereeMutex;

    bool hasReceivedGeom = false;
};

extern IOManager io;

}  // namespace io
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IO_MANAGER_H