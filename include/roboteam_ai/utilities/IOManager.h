#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <proto/SimulationConfiguration.pb.h>
#include <utilities/Constants.h>
#include <utilities/Settings.h>
#include <world/Field.h>

#include <RobotCommandsNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <SimulationConfigurationNetworker.hpp>
#include <WorldNetworker.hpp>
#include <iostream>
#include <mutex>

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
    std::unique_ptr<rtt::net::SimulationConfigurationPublisher> simulationConfigurationPublisher;
    void handleState(const proto::State& state);

    std::unique_ptr<rtt::net::RobotCommandsBluePublisher> robotCommandsBluePublisher;
    std::unique_ptr<rtt::net::RobotCommandsYellowPublisher> robotCommandsYellowPublisher;

    // Only the primary AI publishes settings. The secondary AI subscribes to those settings so they are on the same line
    std::unique_ptr<rtt::net::SettingsPublisher> settingsPublisher;
    std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber;

    rtt::ai::Pause* pause;

    void addCameraAngleToRobotCommands(rtt::RobotCommands& robotCommands);
    bool publishRobotCommands(const rtt::RobotCommands& robotCommands, bool isForTeamYellow);

   public:
    void publishAllRobotCommands(rtt::RobotCommands& robotCommands);
    void publishSettings(const Settings& settings);
    void onSettingsOfPrimaryAI(const proto::Setting& settings);
    // Returns success. Only Primary AI is allowed to send simulation configuration
    bool sendSimulationConfiguration(const proto::SimulationConfiguration& configuration);

    bool init(bool isPrimaryAI);
    proto::State getState();

    bool obtainTeamColorChannel(bool yellowChannel);

    std::mutex stateMutex;
};

extern IOManager io;

}  // namespace io
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IO_MANAGER_H