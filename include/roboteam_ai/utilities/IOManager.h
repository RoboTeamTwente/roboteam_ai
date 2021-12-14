#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <RobotCommandsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <SettingsNetworker.hpp>

#include <utils/Pair.hpp>

#include <iostream>
#include <mutex>

#include "utilities/Constants.h"
#include "world/Field.h"

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
    std::unique_ptr<rtt::net::WorldSubscriber> worldSubscriber;
    void handleState(const proto::State &state);

    std::unique_ptr<rtt::net::RobotCommandsBluePublisher> robotCommandsBluePublisher;
    std::unique_ptr<rtt::net::RobotCommandsYellowPublisher> robotCommandsYellowPublisher;
    std::unique_ptr<rtt::net::SettingsPublisher> settingsPublisher;

    rtt::ai::Pause *pause;

    rtt::net::utils::PairReceiver<16970> *central_server_connection;

    bool publishRobotCommands(const proto::AICommand& aiCommand, bool forTeamYellow);

   public:
    ~IOManager();
    explicit IOManager() = default;
    void publishAllRobotCommands(const std::vector<proto::RobotCommand> &vector);
    void publishSettings(proto::Setting setting);
    void handleCentralServerConnection();
    void init(int teamId);
    proto::State getState();

    std::mutex stateMutex;
};

extern IOManager io;

}  // namespace io
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IO_MANAGER_H