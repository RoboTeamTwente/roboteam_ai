#include "utilities/IOManager.h"

#include "interface/api/Input.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "utilities/Settings.h"
#include "utilities/normalize.h"
#include "world/World.hpp"

namespace rtt::ai::io {

IOManager io;

bool IOManager::init(bool isPrimaryAI) {
    RTT_INFO("Setting up IO networkers as ", isPrimaryAI ? "Primary" : "Secondary", "AI")
    bool success = true;

    auto worldCallback = std::bind(&IOManager::handleState, this, std::placeholders::_1);
    this->worldSubscriber = std::make_unique<rtt::net::WorldSubscriber>(worldCallback);
    simulationConfigurationPublisher = std::make_unique<rtt::net::SimulationConfigurationPublisher>();

    if (isPrimaryAI) {
        try {
            this->settingsPublisher = std::make_unique<rtt::net::SettingsPublisher>();
        } catch (zmqpp::zmq_internal_exception e) {
            success = false;
            RTT_ERROR("Failed to open settings publisher channel. Is it already taken?")
        }
    }
    return success;
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(const proto::State& stateMsg) {
    std::unique_lock<std::mutex> lock(stateMutex);  // write lock
    this->state.CopyFrom(stateMsg);
    if (state.has_referee()) {
        // Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
        std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
        if (state.referee().yellow().name() == ROBOTEAM_TWENTE) {
            SETTINGS.setYellow(true);
        } else if (state.referee().blue().name() == ROBOTEAM_TWENTE) {
            SETTINGS.setYellow(false);
        }
        SETTINGS.setLeft(!(state.referee().blue_team_on_positive_half() ^ SETTINGS.isYellow()));
        if (!SETTINGS.isLeft()) roboteam_utils::rotate(state.mutable_referee());
        auto const& [_, data] = World::instance();
        ai::GameStateManager::setRefereeData(state.referee(), data);
    }
}

void IOManager::publishSettings(proto::Setting setting) {
    if (this->settingsPublisher != nullptr) {
        this->settingsPublisher->publish(setting);
    }
}

void IOManager::publishAllRobotCommands(const std::vector<proto::RobotCommand>& robotCommands) {
    if (!pause->getPause()) {
        proto::AICommand command;
        for (const auto& robotCommand : robotCommands) {
            proto::RobotCommand* protoCommand = command.mutable_commands()->Add();
            protoCommand->CopyFrom(robotCommand);
        }
        command.mutable_extrapolatedworld()->CopyFrom(getState().command_extrapolated_world());
        this->publishRobotCommands(command, SETTINGS.isYellow());
    }
}

bool IOManager::publishRobotCommands(const proto::AICommand& aiCommand, bool forTeamYellow) {
    bool sentCommands = false;

    if (forTeamYellow && this->robotCommandsYellowPublisher != nullptr) {
        sentCommands = this->robotCommandsYellowPublisher->publish(aiCommand);
    } else if (!forTeamYellow && this->robotCommandsBluePublisher != nullptr) {
        sentCommands = this->robotCommandsBluePublisher->publish(aiCommand);
    }

    if (!sentCommands) {
        RTT_ERROR("Failed to send command: Publisher is not initialized (yet)");
    }

    return sentCommands;
}

proto::State IOManager::getState() {
    std::lock_guard<std::mutex> lock(stateMutex);  // read lock
    proto::State copy = state;
    return copy;
}

bool IOManager::switchTeamColorChannel(bool toYellowChannel) {
    bool switchedSuccesfully = false;

    if (toYellowChannel) {
        try {
            this->robotCommandsYellowPublisher = std::make_unique<rtt::net::RobotCommandsYellowPublisher>();
            this->robotCommandsBluePublisher = nullptr;
            switchedSuccesfully = true;
        } catch (zmqpp::zmq_internal_exception e) {
            this->robotCommandsYellowPublisher = nullptr;
        }
    } else {
        try {
            this->robotCommandsBluePublisher = std::make_unique<rtt::net::RobotCommandsBluePublisher>();
            this->robotCommandsYellowPublisher = nullptr;
            switchedSuccesfully = true;
        } catch (zmqpp::zmq_internal_exception e) {
            this->robotCommandsBluePublisher = nullptr;
        }
    }

    return switchedSuccesfully;
}

void IOManager::sendSimulationConfiguration(const proto::SimulationConfiguration &configuration){
    simulationConfigurationPublisher->publish(configuration);
}

}  // namespace rtt::ai::io
