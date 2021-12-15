#include "utilities/IOManager.h"

#include "interface/api/Input.h"
#include "utilities/normalize.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai::io {

IOManager io;

bool IOManager::init(bool isPrimaryAI) {
    RTT_INFO("Setting up IO networkers as ", isPrimaryAI ? "Primary" : "Secondary", "AI")
    bool success = true;
    
    auto worldCallback = std::bind(&IOManager::handleState, this, std::placeholders::_1);
    this->worldSubscriber = std::make_unique<rtt::net::WorldSubscriber>(worldCallback);

    if (isPrimaryAI) {
        try {
            this->settingsPublisher = std::make_unique<rtt::net::SettingsPublisher>();
        } catch (zmqpp::zmq_internal_exception e) {
            success = false;
            RTT_ERROR("Failed to open settings publisher channel. Is it already taken?")
        }
    }
    this->centralServerConnection = std::make_unique<net::utils::PairReceiver<16970>>();

    return success;
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(const proto::State& stateMsg) {
    std::unique_lock<std::mutex> lock(stateMutex);  // write lock
    this->state.CopyFrom(stateMsg);
    if (state.has_referee()) {
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

void IOManager::handleCentralServerConnection() {
    // first receive any setting changes
    bool received = true;
    int numReceivedMessages = 0;
    while (received) {
        auto receivedUIOptions = this->centralServerConnection->read_next<proto::UiSettings>();
        if (receivedUIOptions.is_ok()) {
            // TODO: process value
            receivedUIOptions.value().PrintDebugString();
            numReceivedMessages++;
        } else {
            received = false;
            // we don't print the errors as they mark there are no more messages
        }
    }
    if (numReceivedMessages > 0) {
        std::cout << "received " << numReceivedMessages << " packets from central server" << std::endl;
    }
    // TODO: actually change settings at the relevant places within our AI
    // TODO: make sure to write/add relevant debug information/visualizations (strategy debug, etc.)
    // then, send the current state once
    proto::ModuleState module_state;
    {
        std::lock_guard<std::mutex> lock(stateMutex);  // read lock
        module_state.mutable_system_state()->mutable_state()->CopyFrom(state);
    }
    this->centralServerConnection->write(module_state, true);
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

}  // namespace rtt::ai::io
