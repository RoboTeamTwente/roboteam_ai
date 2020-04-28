/*
 * Created by mrlukasbos on 9-11-18.
 *
 * Set the refgame state according to referee commands.
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <iostream>
#include <map>
#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>
#include "Constants.h"
#include "RefGameState.h"
#include "roboteam_proto/messages_robocup_ssl_referee.pb.h"

namespace rtt::ai {

class StrategyManager {
   public:
    explicit StrategyManager() = default;
    RefGameState getCurrentRefGameState();
    void setCurrentRefGameState(RefCommand command, proto::SSL_Referee_Stage stage, std::optional<world_new::view::BallView> ballOpt);
    void forceCurrentRefGameState(RefCommand command, std::optional<world_new::view::BallView> ballOpt);
    const RefGameState getRefGameStateForRefCommand(RefCommand command);

   private:
    const std::vector<RefGameState> gameStates = {

        // failsafe: for an undefined refstate everything should halt
        RefGameState(RefCommand::UNDEFINED, "halt_strategy", "halt"),

        RefGameState(RefCommand::NORMAL_START, "normal_play_strategy", "default"),
        RefGameState(RefCommand::FORCED_START, "normal_play_strategy", "default"),
        RefGameState(RefCommand::HALT, "halt_strategy", "halt"),
        RefGameState(RefCommand::STOP, "stop_strategy", "stop"),
        RefGameState(RefCommand::TIMEOUT_US, "time_out_strategy", "stop"),
        RefGameState(RefCommand::TIMEOUT_THEM, "halt_strategy", "halt"),
        RefGameState(RefCommand::GOAL_US, "kickoff_them_formation_strategy", "default"),
        RefGameState(RefCommand::GOAL_THEM, "kickoff_us_formation_strategy", "default"),
        RefGameState(RefCommand::BALL_PLACEMENT_US, "ball_placement_us_strategy", "ballplacement_us"),
        RefGameState(RefCommand::BALL_PLACEMENT_THEM, "ball_placement_them_strategy", "ballplacement_them"),
        RefGameState(RefCommand::DIRECT_FREE_US, "free_kick_shoot_strategy", "default"),
        RefGameState(RefCommand::DIRECT_FREE_THEM, "free_kick_them_strategy", "default"),
        RefGameState(RefCommand::INDIRECT_FREE_US, "free_kick_shoot_strategy", "default"),
        RefGameState(RefCommand::INDIRECT_FREE_THEM, "free_kick_them_strategy", "default"),

        // prepare commands
        // These have a follow up command
        RefGameState(RefCommand::PREPARE_KICKOFF_US, "kickoff_us_formation_strategy", "kickoff", false,
                     RefCommand::DO_KICKOFF),
        RefGameState(RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_formation_strategy", "kickoff", false,
                     RefCommand::DEFEND_KICKOFF),
        RefGameState(RefCommand::PREPARE_PENALTY_US, "penalty_us_prepare_strategy", "default", false,
                     RefCommand::DO_PENALTY),
        RefGameState(RefCommand::PREPARE_PENALTY_THEM, "penalty_them_prepare_strategy", "default", false,
                     RefCommand::DEFEND_PENALTY),

        // These two prepares are 'custom' because the refbox does not have seperate commands even though the rules are different
        RefGameState(RefCommand::PREPARE_SHOOTOUT_US, "time_out_strategy", "default", false, RefCommand::DO_SHOOTOUT),
        RefGameState(RefCommand::PREPARE_SHOOTOUT_THEM, "time_out_strategy", "default", false,
                     RefCommand::DEFEND_SHOOTOUT),
        // follow up commands
        // these are custom commands, called when 'normal play' is called after a prepare_ command
        RefGameState(RefCommand::DO_KICKOFF, "kickoff_shoot_strategy", "default", true),
        RefGameState(RefCommand::DEFEND_KICKOFF, "kickoff_them_strategy", "default", true),
        RefGameState(RefCommand::DO_PENALTY, "penalty_us_shoot_strategy", "default", true),
        RefGameState(RefCommand::DEFEND_PENALTY, "penalty_them_defend_strategy", "default", true),
        RefGameState(RefCommand::DO_SHOOTOUT, "time_out_strategy", "default", true),
        RefGameState(RefCommand::DEFEND_SHOOTOUT, "time_out_strategy", "default", true)};
    RefGameState currentRefGameState = gameStates[0];
    RefCommand currentRefCmd = RefCommand::UNDEFINED;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_STRATEGYMANAGER_H
