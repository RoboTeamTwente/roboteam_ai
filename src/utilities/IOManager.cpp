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
        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        robotCommandPublisher = new proto::Publisher<proto::RobotCommand>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_PRIMARY_CHANNEL);
    }
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(proto::State &stateMsg) {
    std::lock_guard<std::mutex> lock(stateMutex);
    this->state.CopyFrom(stateMsg);
}



void IOManager::publishRobotCommand(proto::RobotCommand cmd, rtt::world::World const* world) {
    if (!pause->getPause()) {
        if (world->getWorld()) {
            // the geneva cannot be received from world, so we set it when it gets sent.
            auto robot = world->getWorld()->getRobotForId(cmd.id(), true);
            if (robot) {
                if (cmd.kicker()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::green, robot->get()->getId(), interface::Drawing::CIRCLES, 36, 36, 8);
                }
                if (cmd.chip_kick_forced()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::green, robot->get()->getId(), interface::Drawing::DOTS, 36, 36, 8);
                }
                if (cmd.chipper()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->get()->getPos()}, Qt::yellow, robot->get()->getId(), interface::Drawing::CIRCLES, 36, 36, 8);
                }
                // robot->setDribblerState(cmd.dribbler());
            }
        }
        // sometimes trees are terminated without having a role assigned.
        // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
        if (cmd.id() >= 0 && cmd.id() < 16) {
            robotCommandPublisher->send(cmd);
        }
    }
}

void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->send(setting); }

    proto::State IOManager::getState() const {
        std::lock_guard<std::mutex> lock(stateMutex);
        return state;
    }
}  // namespace rtt::ai::io
