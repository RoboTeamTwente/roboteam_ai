//
// Created by rolf on 23-10-18.
//

#ifndef ROBOTEAM_AI_REFEREE_HPP
#define ROBOTEAM_AI_REFEREE_HPP

#include "roboteam_msgs/RefereeData.h"
namespace rtt{
namespace ai{
/**
 * /enum RefState
 * /brief Used to hold the referee state.
 *
 * A different entity from the RefState used in the ros messages.
 */
 enum class RefState {
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
class Referee {
    public:
        static roboteam_msgs::RefereeData get();
        /**
         * \brief Sets the refstate.
         *        Only to be used when a new refstate has been received.
         */
        static void set(roboteam_msgs::RefereeData refCommand);

        static bool hasReceivedFirstCommand();
        static RefState getState();

        static boost::optional<RefState> getCurrentRefCommand();
        static boost::optional<RefState> getPreviousRefCommand();

        static bool waitForFirstRefCommand();

    private:
        static roboteam_msgs::RefereeData lastRef;
        static const std::vector<RefStateTransitionFunction> transitions;

        static boost::optional<RefState> previousRefCommand;
        static boost::optional<RefState> currentRefCommand;

};
}
}
#endif //ROBOTEAM_AI_REFEREE_HPP
