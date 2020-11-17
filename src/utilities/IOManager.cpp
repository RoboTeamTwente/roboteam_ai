#include <utilities/IOManager.h>
#include <utilities/Settings.h>

#include <include/roboteam_ai/utilities/GameStateManager.hpp>

#include "interface/api/Input.h"
#include "roboteam_utils/normalize.h"
#include "utilities/Pause.h"
#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::io {

std::mutex IOManager::stateMutex;

IOManager io;

IOManager::~IOManager() {
    delete worldSubscriber;
    delete robotCommandPublisher;
    delete settingsPublisher;
}

void IOManager::init(int teamId) {
    RTT_INFO("Setting up IO publishers/subscribers")
    worldSubscriber = new proto::Subscriber<proto::State>(proto::WORLD_CHANNEL, &IOManager::handleState, this);

    // set up advertisement to publish robotcommands and settings
    if (teamId == 1) {
        robotCommandPublisher = new proto::Publisher<proto::AICommand>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        robotCommandPublisher = new proto::Publisher<proto::AICommand>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_PRIMARY_CHANNEL);
    }
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(proto::State &stateMsg) {
    std::lock_guard<std::mutex> lock(stateMutex);
    this->state.CopyFrom(stateMsg);
    if(state.has_referee()){
        roboteam_utils::rotate(state.mutable_referee());
        // Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
        std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
        if (state.referee().yellow().name() == ROBOTEAM_TWENTE) {
            SETTINGS.setYellow(true);
        } else if (state.referee().blue().name() == ROBOTEAM_TWENTE) {
            SETTINGS.setYellow(false);
        }
        SETTINGS.setLeft(!(state.referee().blue_team_on_positive_half() ^ SETTINGS.isYellow()));
        auto const& [_, data] = World::instance();
        ai::GameStateManager::setRefereeData(state.referee(), data);

    }
}


void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->send(setting); }

void IOManager::publishAllRobotCommands(const std::vector<proto::RobotCommand>& robotCommands) {
    if(!pause->getPause()) {
        proto::AICommand command;
        for(const auto& robotCommand : robotCommands){
          proto::RobotCommand * protoCommand = command.mutable_commands()->Add();
          protoCommand->CopyFrom(robotCommand);
        }
        command.mutable_extrapolatedworld()->CopyFrom(getState().command_extrapolated_world());
        robotCommandPublisher->send(command);
    }
}
proto::State IOManager::getState() const {
        std::lock_guard<std::mutex> lock(stateMutex);
        return state;
    }
}  // namespace rtt::ai::io
