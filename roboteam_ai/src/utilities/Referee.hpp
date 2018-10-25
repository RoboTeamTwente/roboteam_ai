//
// Created by rolf on 23-10-18.
//

#ifndef ROBOTEAM_AI_REFEREE_HPP
#define ROBOTEAM_AI_REFEREE_HPP

#include "roboteam_msgs/RefereeData.h"
#include "boost/optional.hpp"
#include "ros/ros.h"
namespace rtt{
namespace ai{
/**
 * /enum RefState
 * /brief Used to hold the referee state.
 *
 * A different entity from the RefState used in the ros messages.
 */
 enum class RefGameState {
        // Ref states as dictated by RoboCup SSL
        // (with corresponding numeral identifiers - feel free to convert to and from)
        HALT = 0,
        STOP = 1,
        NORMAL_START = 2,
        FORCED_START = 3,
        PREPARE_KICKOFF_US = 4,
        PREPARE_KICKOFF_THEM = 5,
        PREPARE_PENALTY_US = 6,
        PREPARE_PENALTY_THEM = 7,
        DIRECT_FREE_US = 8,
        DIRECT_FREE_THEM = 9,
        INDIRECT_FREE_US = 10,
        INDIRECT_FREE_THEM = 11,
        TIMEOUT_US = 12,
        TIMEOUT_THEM = 13,
        GOAL_US = 14,
        GOAL_THEM = 15,
        BALL_PLACEMENT_US = 16,
        BALL_PLACEMENT_THEM = 17,

        // Custom extended refstates
        // (notice they have no numeral identifiers - don't use them like that)
        DO_KICKOFF,
        DEFEND_KICKOFF,
        DO_PENALTY,
        DEFEND_PENALTY,
                };
const std::map < int ,std::string > refstagelookup = {
            {0,"NORMAL_FIRST_HALF_PRE"},
            {1,"NORMAL_FIRST_HALF"},
            {2,"NORMAL_HALF_TIME"},
            {3,"NORMAL_SECOND_HALF_PRE"},
            {4,"NORMAL_SECOND_HALF"},
            {5,"EXTRA_TIME_BREAK"},
            {6,"EXTRA_FIRST_HALF_PRE"},
            {7,"EXTRA_FIRST_HALF"},
            {8,"EXTRA_HALF_TIME"},
            {9,"EXTRA_SECOND_HALF_PRE"},
            {10,"EXTRA_SECOND_HALF"},
            {11,"PENALTY_SHOOTOUT_BREAK"},
            {12,"PENALTY_SHOOTOUT"},
            {13,"POST_GAME"}
    };

//Do not touch this unless you want to open pandora's box.
 std::vector<RefGameState> const ALL_REFSTATES = {
            RefGameState::HALT,
            RefGameState::STOP,
            RefGameState::NORMAL_START,
            RefGameState::FORCED_START,
            RefGameState::PREPARE_KICKOFF_US,
            RefGameState::PREPARE_KICKOFF_THEM,
            RefGameState::PREPARE_PENALTY_US,
            RefGameState::PREPARE_PENALTY_THEM,
            RefGameState::DIRECT_FREE_US,
            RefGameState::DIRECT_FREE_THEM,
            RefGameState::INDIRECT_FREE_US,
            RefGameState::INDIRECT_FREE_THEM,
            RefGameState::TIMEOUT_US,
            RefGameState::TIMEOUT_THEM,
            RefGameState::GOAL_US,
            RefGameState::GOAL_THEM,
            RefGameState::BALL_PLACEMENT_US,
            RefGameState::BALL_PLACEMENT_THEM,

            RefGameState::DO_KICKOFF,
            RefGameState::DEFEND_KICKOFF,
            RefGameState::DO_PENALTY,
            RefGameState::DEFEND_PENALTY,
    } ;

    std::string refStateToString(RefGameState s);
    boost::optional<RefGameState> stringToRefState(std::string s);
    boost::optional<RefGameState> toRefState(int refStateInt);
    boost::optional<int> fromRefState(RefGameState refState);
/**
 * \class Referee
 * \brief Contains the latest 2 referee messages and always returns our (custom) RefState for these scenario's
 *
 */
class Referee {
    //Keeps track of the previous 2 Referee Commands to make sure we are not in a twoState. (see a refbox diagram on drive).
    //lastRef is the last command we received. TimeLeft just keeps track of the time
private:
    static boost::optional<RefGameState> previousRefCommand;
    static boost::optional<RefGameState> currentRefCommand;
    static roboteam_msgs::RefereeData lastRef;
    static std::map<std::string,int> timeLeft; //tracks per stage what the last timeLeft was we received for it.

public:
        static boost::optional<RefGameState> getFirstState(boost::optional<RefGameState> previousCmdOpt, RefGameState currentCmd);
        static boost::optional<RefGameState> getFirstState();
        static RefGameState getExtendedState();
        static bool isTwoState(boost::optional<RefGameState> previousCmdOpt, RefGameState currentCmd);

        static roboteam_msgs::RefereeData get();
        static RefGameState getState();
        /**
         * \brief Sets the refstate.
         *        Only to be used when a new refstate has been received.
         */
        static void set(roboteam_msgs::RefereeData refCommand);

        static bool hasReceivedFirstCommand();

        static boost::optional<RefGameState> getCurrentRefCommand();
        static boost::optional<RefGameState> getPreviousRefCommand();

        static bool waitForFirstRefCommand();

        static int getTimeLeft(std::string gameStage);

        /// \brief returns OUR_SCORE, THEIR_SCORE
        static std::pair<int,int> currentScore();

};
}
}
#endif //ROBOTEAM_AI_REFEREE_HPP
