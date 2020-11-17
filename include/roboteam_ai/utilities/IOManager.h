#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include <mutex>

#include <roboteam_proto/State.pb.h>
#include <roboteam_proto/AICommand.pb.h>
#include <roboteam_proto/Setting.pb.h>

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
    proto::State state;

    proto::Subscriber<proto::State> *worldSubscriber;
    void handleState(proto::State &state);


    proto::Publisher<proto::AICommand> *robotCommandPublisher;
    proto::Publisher<proto::Setting> *settingsPublisher;

    rtt::ai::Pause *pause;

   public:
    ~IOManager();
    explicit IOManager() = default;
    void publishAllRobotCommands(const std::vector<proto::RobotCommand>& vector);
    void publishSettings(proto::Setting setting);
    void init(int teamId);
    proto::State getState() const;

    static std::mutex stateMutex;

};

extern IOManager io;

}  // namespace io
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IO_MANAGER_H