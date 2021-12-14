#include "utilities/IOManager.h"

#include "interface/api/Input.h"
#include "utilities/normalize.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai::io {

IOManager io;

IOManager::~IOManager() {
    this->worldSubscriber = nullptr;
    this->robotCommandsBluePublisher = nullptr;
    this->robotCommandsYellowPublisher = nullptr;
    this->settingsPublisher = nullptr;
    //this->central_server_connection;
}

void IOManager::init(int teamId) {
    RTT_INFO("Setting up IO publishers/subscribers")
    auto worldCallback = std::bind(&IOManager::handleState, this, std::placeholders::_1);
    this->worldSubscriber = std::make_unique<rtt::net::WorldSubscriber>(worldCallback);

    this->settingsPublisher = std::make_unique<rtt::net::SettingsPublisher>();

    //central_server_connection = new networking::PairReceiver<16970>();
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

void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->publish(setting); }

void IOManager::publishAllRobotCommands(const std::vector<proto::RobotCommand>& robotCommands) {
    if (!pause->getPause()) {
        proto::AICommand command;
        for (const auto& robotCommand : robotCommands) {
            proto::RobotCommand* protoCommand = command.mutable_commands()->Add();
            protoCommand->CopyFrom(robotCommand);
        }
        command.mutable_extrapolatedworld()->CopyFrom(getState().command_extrapolated_world());
        this->robotCommandsBluePublisher->publish(command);
    }
}

bool IOManager::publishRobotCommands(const proto::AICommand& aiCommand, bool forTeamYellow) {
    if (forTeamYellow) {
        if (this->robotCommandsYellowPublisher != nullptr) {
            this->robotCommandsYellowPublisher->publish(aiCommand);
            return true;
        }
    } else {
        if (this->robotCommandsBluePublisher != nullptr) {
            this->robotCommandsBluePublisher->publish(aiCommand);
            return true;
        }
    }
    return false;
}

void IOManager::handleCentralServerConnection() {
    // first receive any setting changes
    bool received = true;
    int numReceivedMessages = 0;
    while (received) {
        auto receivedUIOptions = central_server_connection->read_next<proto::UiSettings>();
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
    central_server_connection->write(module_state, true);
}
proto::State IOManager::getState() {
    std::lock_guard<std::mutex> lock(stateMutex);  // read lock
    proto::State copy = state;
    return copy;
}
}  // namespace rtt::ai::io
