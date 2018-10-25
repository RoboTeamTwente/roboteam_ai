//
// Created by rolf on 23-10-18.
//

#include "Referee.hpp"
namespace rtt{
namespace ai{
std::string refStateToString(RefGameState s) {
        if (s == RefGameState::HALT) {
            return "HALT";
        } else if (s == RefGameState::STOP) {
            return "STOP";
        } else if (s == RefGameState::NORMAL_START) {
            return "NORMAL_START";
        } else if (s == RefGameState::FORCED_START) {
            return "FORCED_START";
        } else if (s == RefGameState::PREPARE_KICKOFF_US) {
            return "PREPARE_KICKOFF_US";
        } else if (s == RefGameState::PREPARE_KICKOFF_THEM) {
            return "PREPARE_KICKOFF_THEM";
        } else if (s == RefGameState::PREPARE_PENALTY_US) {
            return "PREPARE_PENALTY_US";
        } else if (s == RefGameState::PREPARE_PENALTY_THEM) {
            return "PREPARE_PENALTY_THEM";
        } else if (s == RefGameState::DIRECT_FREE_US) {
            return "DIRECT_FREE_US";
        } else if (s == RefGameState::DIRECT_FREE_THEM) {
            return "DIRECT_FREE_THEM";
        } else if (s == RefGameState::INDIRECT_FREE_US) {
            return "INDIRECT_FREE_US";
        } else if (s == RefGameState::INDIRECT_FREE_THEM) {
            return "INDIRECT_FREE_THEM";
        } else if (s == RefGameState::TIMEOUT_US) {
            return "TIMEOUT_US";
        } else if (s == RefGameState::TIMEOUT_THEM) {
            return "TIMEOUT_THEM";
        } else if (s == RefGameState::GOAL_US) {
            return "GOAL_US";
        } else if (s == RefGameState::GOAL_THEM) {
            return "GOAL_THEM";
        } else if (s == RefGameState::BALL_PLACEMENT_US) {
            return "BALL_PLACEMENT_US";
        } else if (s == RefGameState::BALL_PLACEMENT_THEM) {
            return "BALL_PLACEMENT_THEM";
        } else if (s == RefGameState::DO_KICKOFF) {
            return "DO_KICKOFF";
        } else if (s == RefGameState::DEFEND_KICKOFF) {
            return "DEFEND_KICKOFF";
        } else if (s == RefGameState::DO_PENALTY) {
            return "DO_PENALTY";
        } else if (s == RefGameState::DEFEND_PENALTY) {
            return "DEFEND_PENALTY";
        } else {
            return "UNKNOWN REF STATE";
        }
    }
boost::optional<RefGameState> stringToRefState(std::string s) {
        if (s == "HALT") {
            return RefGameState::HALT;
        } else if (s == "STOP") {
            return RefGameState::STOP;
        } else if (s == "NORMAL_START") {
            return RefGameState::NORMAL_START;
        } else if (s == "FORCED_START") {
            return RefGameState::FORCED_START;
        } else if (s == "PREPARE_KICKOFF_US") {
            return RefGameState::PREPARE_KICKOFF_US;
        } else if (s == "PREPARE_KICKOFF_THEM") {
            return RefGameState::PREPARE_KICKOFF_THEM;
        } else if (s == "PREPARE_PENALTY_US") {
            return RefGameState::PREPARE_PENALTY_US;
        } else if (s == "PREPARE_PENALTY_THEM") {
            return RefGameState::PREPARE_PENALTY_THEM;
        } else if (s == "DIRECT_FREE_US") {
            return RefGameState::DIRECT_FREE_US;
        } else if (s == "DIRECT_FREE_THEM") {
            return RefGameState::DIRECT_FREE_THEM;
        } else if (s == "INDIRECT_FREE_US") {
            return RefGameState::INDIRECT_FREE_US;
        } else if (s == "INDIRECT_FREE_THEM") {
            return RefGameState::INDIRECT_FREE_THEM;
        } else if (s == "TIMEOUT_US") {
            return RefGameState::TIMEOUT_US;
        } else if (s == "TIMEOUT_THEM") {
            return RefGameState::TIMEOUT_THEM;
        } else if (s == "GOAL_US") {
            return RefGameState::GOAL_US;
        } else if (s == "GOAL_THEM") {
            return RefGameState::GOAL_THEM;
        } else if (s == "BALL_PLACEMENT_US") {
            return RefGameState::BALL_PLACEMENT_US;
        } else if (s == "BALL_PLACEMENT_THEM") {
            return RefGameState::BALL_PLACEMENT_THEM;
        } else if (s == "DO_KICKOFF") {
            return RefGameState::DO_KICKOFF;
        } else if (s == "DEFEND_KICKOFF") {
            return RefGameState::DEFEND_KICKOFF;
        } else if (s == "DO_PENALTY") {
            return RefGameState::DO_PENALTY;
        } else if (s == "DEFEND_PENALTY") {
            return RefGameState::DEFEND_PENALTY;
        } else {
            return boost::none;
        }
    }
boost::optional<RefGameState> toRefState(int refStateInt) {
        if (refStateInt >= 0 && refStateInt <= 17) {
            return static_cast<RefGameState>(refStateInt);
        }

        return boost::none;
    }
boost::optional<int> fromRefState(RefGameState refState) {
        int rawRefState = static_cast<int>(refState);

        if (rawRefState >= 0 && rawRefState < 17) {
            return rawRefState;
        }

        return boost::none;
    }

// REFEREE FUNCTIONALITY
// Initialize static variables
roboteam_msgs::RefereeData Referee::lastRef;
boost::optional<RefGameState> Referee::previousRefCommand;
boost::optional<RefGameState> Referee::currentRefCommand;
std::map<std::string,int> Referee::timeLeft;

roboteam_msgs::RefereeData Referee::get(){
    return Referee::lastRef;
}

RefGameState Referee::getState() {
        if (currentRefCommand) {
            return *currentRefCommand;
        } else {
            // Something bad probably happens here
            return static_cast<RefGameState>(Referee::lastRef.command.command);
            // Might as well launch a nuclear missile
        }
    }

void Referee::set(roboteam_msgs::RefereeData refCommand){
    auto previousCurrentRefCommand = currentRefCommand;

    if (auto refStateOpt = toRefState(refCommand.command.command)) {
        currentRefCommand = *refStateOpt;
    } else {
        ROS_ERROR("Received a refstate that is not a refstate! I.e. is more or less than 0 - 17 inclusive. Refstate: %d", refCommand.command.command);
        return;
    }


    if (currentRefCommand != previousCurrentRefCommand) {
        previousRefCommand = previousCurrentRefCommand;
    }
    // insert stage with stage_time_left in our map.
    timeLeft[refstagelookup.at(refCommand.stage.stage)]=refCommand.stage_time_left;
    Referee::lastRef = refCommand;
}

boost::optional<RefGameState> Referee::getCurrentRefCommand() {
        return currentRefCommand;
    }

boost::optional<RefGameState> Referee::getPreviousRefCommand() {
        return previousRefCommand;
    }

bool Referee::waitForFirstRefCommand() {
        ros::Rate fps60(60);

        while (!(hasReceivedFirstCommand())) {
            fps60.sleep();
            ros::spinOnce();

            ROS_INFO_THROTTLE(1, "Waiting for first ref command...");

            if (!ros::ok()) {
                return false;
            }
        }

        return true;
    }

bool Referee::hasReceivedFirstCommand() {
        return !!currentRefCommand;
    }

// The TwoState Pairs that we have custom refStates for. Each of the last of these two referee states is split into a custom referee-state and Normal Play in our code.
const std::map<std::pair<boost::optional<RefGameState>, RefGameState>, RefGameState> twoStatePairs = {
            { {RefGameState::PREPARE_KICKOFF_US, RefGameState::NORMAL_START},
                    RefGameState::DO_KICKOFF
            },
            { {RefGameState::PREPARE_KICKOFF_THEM, RefGameState::NORMAL_START},
                    RefGameState::DEFEND_KICKOFF
            },
            { {RefGameState::PREPARE_PENALTY_US, RefGameState::NORMAL_START},
                    RefGameState::DO_PENALTY
            },
            { {RefGameState::PREPARE_PENALTY_THEM, RefGameState::NORMAL_START},
                    RefGameState::DEFEND_PENALTY
            },
            { {boost::none, RefGameState::INDIRECT_FREE_US},
                    RefGameState::INDIRECT_FREE_US
            },
            { {boost::none, RefGameState::INDIRECT_FREE_THEM},
                    RefGameState::INDIRECT_FREE_THEM
            },
            { {boost::none, RefGameState::DIRECT_FREE_US},
                    RefGameState::DIRECT_FREE_US
            },
            { {boost::none, RefGameState::DIRECT_FREE_THEM},
                    RefGameState::DIRECT_FREE_THEM
            },
    } ;

/// Checks if a pair is a TwoState
bool Referee::isTwoState(boost::optional<RefGameState> previousCmdOpt, RefGameState currentCmd) {
        return twoStatePairs.find({previousCmdOpt, currentCmd}) != twoStatePairs.end()
               || twoStatePairs.find({boost::none, currentCmd}) != twoStatePairs.end()
                ;
}

/**
 *  Returns the first stage from the two stage pair previousCmd and currentCmd.
 *  The second stage is always normal play.
 */
    boost::optional<RefGameState> Referee::getFirstState(boost::optional<RefGameState> previousCmdOpt, RefGameState currentCmd) {
        auto stateIt = twoStatePairs.find({previousCmdOpt, currentCmd});
        if (stateIt != twoStatePairs.end()) {
            return stateIt->second;
        }
        else {
            stateIt = twoStatePairs.find({boost::none, currentCmd});
            if (stateIt != twoStatePairs.end()) {
                return stateIt->second;
            }
        }
        return boost::none;
    }

    boost::optional<RefGameState> Referee::getFirstState() {
        return getFirstState( Referee::getPreviousRefCommand()
                , Referee::getState()
        );
    }

    RefGameState Referee::getExtendedState() {
        if (auto possibleFirstState = getFirstState()) {
            return *possibleFirstState;
        } else {
            return Referee::getState();
        }
    }

    int Referee::getTimeLeft(std::string gameStage) {
        return timeLeft[gameStage];
    }
    std::pair<int,int> Referee::currentScore(){
        std::pair<int,int> output(Referee::get().us.score,Referee::get().them.score);
        return output;
    }
}
}