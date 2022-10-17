#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai {

proto::SSL_Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;

proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(proto::SSL_Referee refMsg, const rtt_world::World* data) {
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    RefCommand cmd;
    // COLOR DEPENDENT STATES
    if (SETTINGS.isYellow()) {
        switch (refMsg.command()) {
            case proto::SSL_Referee_Command_HALT:
                cmd = RefCommand::HALT;
                break;
            case proto::SSL_Referee_Command_STOP:
                cmd = RefCommand::STOP;
                break;
            case proto::SSL_Referee_Command_NORMAL_START:
                cmd = RefCommand::NORMAL_START;
                break;
            case proto::SSL_Referee_Command_FORCE_START:
                cmd = RefCommand::FORCED_START;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                cmd = RefCommand::PREPARE_KICKOFF_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                cmd = RefCommand::PREPARE_KICKOFF_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                cmd = RefCommand::PREPARE_PENALTY_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                cmd = RefCommand::PREPARE_SHOOTOUT_THEM;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW:
                cmd = RefCommand::DIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_BLUE:
                cmd = RefCommand::DIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                cmd = RefCommand::INDIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE:
                cmd = RefCommand::INDIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_YELLOW:
                cmd = RefCommand::TIMEOUT_US;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_BLUE:
                cmd = RefCommand::TIMEOUT_THEM;
                break;
            // case proto::SSL_Referee_Command_GOAL_YELLOW: // TODO: Change these deprecated fields
            //     cmd = RefCommand::GOAL_US;
            //     break;
            // case proto::SSL_Referee_Command_GOAL_BLUE:
            //     cmd = RefCommand::GOAL_THEM;
            //     break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                cmd = RefCommand::BALL_PLACEMENT_US;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                cmd = RefCommand::BALL_PLACEMENT_THEM;
                break;
            default: {
                RTT_ERROR("Unknown refstate, halting all robots for safety!")
                cmd = RefCommand::HALT;
                break;
            }
        }
    } else {
        switch (refMsg.command()) {
            case proto::SSL_Referee_Command_HALT:
                cmd = RefCommand::HALT;
                break;
            case proto::SSL_Referee_Command_STOP:
                cmd = RefCommand::STOP;
                break;
            case proto::SSL_Referee_Command_NORMAL_START:
                cmd = RefCommand::NORMAL_START;
                break;
            case proto::SSL_Referee_Command_FORCE_START:
                cmd = RefCommand::FORCED_START;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                cmd = RefCommand::PREPARE_KICKOFF_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                cmd = RefCommand::PREPARE_KICKOFF_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                cmd = RefCommand::PREPARE_PENALTY_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                cmd = RefCommand::PREPARE_SHOOTOUT_US;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW:
                cmd = RefCommand::DIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_BLUE:
                cmd = RefCommand::DIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                cmd = RefCommand::INDIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE:
                cmd = RefCommand::INDIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_YELLOW:
                cmd = RefCommand::TIMEOUT_THEM;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_BLUE:
                cmd = RefCommand::TIMEOUT_US;
                break;
            // case proto::SSL_Referee_Command_GOAL_YELLOW: //TODO: Fix replacements for these
            //     cmd = RefCommand::GOAL_THEM;
            //     break;
            // case proto::SSL_Referee_Command_GOAL_BLUE:
            //     cmd = RefCommand::GOAL_US;
            //     break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                cmd = RefCommand::BALL_PLACEMENT_THEM;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                cmd = RefCommand::BALL_PLACEMENT_US;
                break;
            default: {
                RTT_ERROR("Unknown refstate, halting all robots for safety!")
                cmd = RefCommand::HALT;
                break;
            }
        }
    }

    auto stage = refMsg.stage();
    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentRefGameState(cmd, stage, world->getBall());
    }
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (interface::Output::usesRefereeCommands()) {
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());

        if (SETTINGS.isYellow()) {
            newGameState.keeperId = getRefereeData().yellow().goalkeeper();
        } else {
            newGameState.keeperId = getRefereeData().blue().goalkeeper();
        }
        // if there is a ref we set the interface gamestate to these values as well
        // this makes sure that when we stop using the referee we don't return to an unknown state,
        // // so now we keep the same.
        interface::Output::setInterfaceGameState(newGameState);
    } else {
        newGameState = interface::Output::getInterfaceGameState();
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd, std::optional<rtt_world::view::BallView> ball) {
    RTT_INFO("Forcing new refstate!")

    // overwrite both the interface and the strategy manager.
    interface::Output::setInterfaceGameState(strategymanager.getRefGameStateForRefCommand(cmd));

    strategymanager.forceCurrentRefGameState(cmd, ball);
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
    return Vector2(designatedPos.x() / 1000, designatedPos.y() / 1000);
}

void GameStateManager::updateInterfaceGameState(const char* name) {
    if (strcmp(name, "Ball Placement Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("ball_placement_us", "ballplacement_us"));
    } else if (strcmp(name, "Halt") == 0) {
        interface::Output::setInterfaceGameState(GameState("halt", "halt"));
    } else if (strcmp(name, "Free Kick Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_them", "stop"));
    } else if (strcmp(name, "Free Kick Us At Goal") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_us", "default"));
    } else if (strcmp(name, "Free Kick Us Pass") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_us", "default"));
    } else if (strcmp(name, "Ball Placement Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("ball_placement_them", "ballplacement_them"));
    } else if (strcmp(name, "Kick Off Them Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_them_prepare", "kickoff"));
    } else if (strcmp(name, "Kick Off Us Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_us_prepare", "kickoff"));
    } else if (strcmp(name, "Kick Off Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_them", "default"));
    } else if (strcmp(name, "Kick Off Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_us", "default"));
    } else if (strcmp(name, "Penalty Them Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_them_prepare", "default"));
    } else if (strcmp(name, "Penalty Us Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_us_prepare", "default"));
    } else if (strcmp(name, "Penalty Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_them", "default"));
    } else if (strcmp(name, "Penalty Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_us", "default"));
    } else if (strcmp(name, "Time Out") == 0) {
        interface::Output::setInterfaceGameState(GameState("time_out", "default"));
    } else if (strcmp(name, "Defensive Stop Formation") == 0) {
        interface::Output::setInterfaceGameState(GameState("stop", "stop"));
    } else if (strcmp(name, "Aggressive Stop Formation") == 0) {
        interface::Output::setInterfaceGameState(GameState("stop", "stop"));
    } else {
        RTT_WARNING("Play has been selected for which no ruleset is found!");
        interface::Output::setInterfaceGameState(GameState("normal_play", "default"));
    }
}
}  // namespace rtt::ai
