/*
 * Created by mrlukasbos on 9-11-18.
 *
 * Set the refgame state according to referee commands.
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <proto/messages_robocup_ssl_referee.pb.h>

#include <iostream>
#include <map>

#include "Constants.h"
#include "RefGameState.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {

class StrategyManager {
   public:
    explicit StrategyManager() = default;
    RefGameState getCurrentRefGameState();
    void setCurrentRefGameState(RefCommand command, proto::SSL_Referee_Stage stage, std::optional<rtt::world::view::BallView> ballOpt);
    void forceCurrentRefGameState(RefCommand command, std::optional<rtt::world::view::BallView> ballOpt);
    const RefGameState getRefGameStateForRefCommand(RefCommand command);

   private:
    const std::vector<RefGameState> gameStates = {

        // failsafe: for an undefined refstate everything should halt
        RefGameState(RefCommand::UNDEFINED, "halt", "halt"),

        RefGameState(RefCommand::NORMAL_START, "normal_play", "default"), RefGameState(RefCommand::FORCED_START, "normal_play", "default"),
        RefGameState(RefCommand::HALT, "halt", "halt"), RefGameState(RefCommand::STOP, "ball_placement_them", "ballplacement_them"),
        RefGameState(RefCommand::TIMEOUT_US, "halt", "halt"), RefGameState(RefCommand::TIMEOUT_THEM, "halt", "halt"),
        RefGameState(RefCommand::GOAL_US, "kickoff_them_prepare", "default"), RefGameState(RefCommand::GOAL_THEM, "kickoff_us_prepare", "default"),
        RefGameState(RefCommand::BALL_PLACEMENT_US, "ball_placement_us", "ballplacement_us"),
        RefGameState(RefCommand::BALL_PLACEMENT_THEM, "ball_placement_them", "ballplacement_them"), RefGameState(RefCommand::DIRECT_FREE_US, "free_kick_us", "default"),
        RefGameState(RefCommand::DIRECT_FREE_THEM, "free_kick_them", "stop"), RefGameState(RefCommand::INDIRECT_FREE_US, "free_kick_us", "default"),
        RefGameState(RefCommand::INDIRECT_FREE_THEM, "free_kick_them", "stop"),

        // prepare commands
        // These have a follow up command
        RefGameState(RefCommand::PREPARE_KICKOFF_US, "kickoff_us_prepare", "kickoff", false, RefCommand::DO_KICKOFF),
        RefGameState(RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_prepare", "kickoff", false, RefCommand::DEFEND_KICKOFF),
        RefGameState(RefCommand::PREPARE_PENALTY_US, "penalty_us_prepare", "default", false, RefCommand::DO_PENALTY),
        RefGameState(RefCommand::PREPARE_PENALTY_THEM, "penalty_them_prepare", "default", false, RefCommand::DEFEND_PENALTY),

        // These two prepares are 'custom' because the refbox does not have seperate commands even though the rules are different
        // TODO: When GameController selects penalty, looks like these RefState's are selected.
        // Put this back in again when the PREPARE_SHOOT commands are correctly selected.
        //        RefGameState(RefCommand::PREPARE_SHOOTOUT_US, "time_out", "default", false, RefCommand::DO_SHOOTOUT),
        //        RefGameState(RefCommand::PREPARE_SHOOTOUT_THEM, "time_out", "default", false,
        //                     RefCommand::DEFEND_SHOOTOUT),

        RefGameState(RefCommand::PREPARE_SHOOTOUT_US, "penalty_us_prepare", "default", false, RefCommand::DO_PENALTY),
        RefGameState(RefCommand::PREPARE_SHOOTOUT_THEM, "penalty_them_prepare", "default", false, RefCommand::DEFEND_SHOOTOUT),
        RefGameState(RefCommand::PRE_HALF, "formation_pre_half", "default", false),

        // follow up commands
        // these are custom commands, called when 'normal play' is called after a prepare_ command
        RefGameState(RefCommand::DO_KICKOFF, "kickoff_us", "default", true), RefGameState(RefCommand::DEFEND_KICKOFF, "kickoff_them", "default", true),
        RefGameState(RefCommand::DO_PENALTY, "penalty_us", "default", true), RefGameState(RefCommand::DEFEND_PENALTY, "penalty_them", "default", true),
        RefGameState(RefCommand::DO_SHOOTOUT, "time_out", "default", true), RefGameState(RefCommand::DEFEND_SHOOTOUT, "time_out", "default", true)};

    RefGameState currentRefGameState = gameStates[0];
    RefCommand currentRefCmd = RefCommand::UNDEFINED;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_STRATEGYMANAGER_H
