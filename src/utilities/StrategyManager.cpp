//
// Created by mrlukasbos on 9-11-18.
//

#include "utilities/StrategyManager.h"

#include "stp/constants/ControlConstants.h"
namespace rtt::ai {

// process ref commands
void StrategyManager::setCurrentRefGameState(RefCommand command, proto::SSL_Referee_Stage stage, std::optional<world::view::BallView> ballOpt) {
    // if the stage is shootout, we interpret penalty commands as shootOut penalty commands
    if (stage == proto::SSL_Referee_Stage_PENALTY_SHOOTOUT) {
        if (command == RefCommand::PREPARE_PENALTY_US) {
            command = RefCommand::PREPARE_SHOOTOUT_US;
        } else if (command == RefCommand::PREPARE_PENALTY_THEM) {
            command = RefCommand::PREPARE_SHOOTOUT_THEM;
        }
    }

    // If game is in pre start stage and the game is in stop state
    if (command == RefCommand::STOP && (stage == proto::SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE || stage == proto::SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE ||
                                        stage == proto::SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE || stage == proto::SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE)) {
        command = RefCommand::PRE_HALF;
    }

    // If the ball has been kicked during kickoff or a free kick, continue with NORMAL_START
    if ((currentRefGameState.commandId == RefCommand::DIRECT_FREE_THEM || currentRefGameState.commandId == RefCommand::DIRECT_FREE_US ||
         currentRefGameState.commandId == RefCommand::DO_KICKOFF || currentRefGameState.commandId == RefCommand::DEFEND_KICKOFF ||
         currentRefGameState.commandId == RefCommand::INDIRECT_FREE_US || currentRefGameState.commandId == RefCommand::INDIRECT_FREE_THEM) &&
        ballOpt.has_value() && (ballOpt.value()->velocity.length() > stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT)) {
        RefGameState newState = getRefGameStateForRefCommand(RefCommand::NORMAL_START);
        currentRefGameState = newState;
        currentRefCmd = command;
        return;
    }

    // if the command is the same as the previous, we don't need to do anything
    if (command == currentRefCmd) {
        return;
    }

    // if the command is the same, we don't need to do anything
    if (command == currentRefGameState.commandId) {
        return;
    }

    // otherwise, if we are in a followupstate and the refcommand is normal start we don't change a thing
    if (currentRefGameState.isfollowUpCommand && command == RefCommand::NORMAL_START) {
        return;
    }

    // we need to change refgamestate here
    RefGameState newState;
    if (currentRefGameState.hasFollowUpCommand() && command == RefCommand::NORMAL_START) {
        newState = getRefGameStateForRefCommand(currentRefGameState.followUpCommandId);
    } else {
        newState = getRefGameStateForRefCommand(command);
    }

    currentRefGameState = newState;
    currentRefCmd = command;
}

RefGameState StrategyManager::getCurrentRefGameState() { return currentRefGameState; }

const RefGameState StrategyManager::getRefGameStateForRefCommand(RefCommand command) {
    for (auto gameState : this->gameStates) {
        if (gameState.commandId == command) {
            return gameState;
        }
    }
    std::cerr << "Returning an undefined refstate! This should never happen!" << std::endl;
    return gameStates[0];
}

void StrategyManager::forceCurrentRefGameState(RefCommand command, std::optional<world::view::BallView> ballOpt) {
    // we need to change refgamestate here
    RefGameState newState = getRefGameStateForRefCommand(command);

    currentRefGameState = newState;
}

}  // namespace rtt::ai
