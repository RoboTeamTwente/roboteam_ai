//
// Created by maxl on 11-02-21.
//

#ifndef RTT_PASSCOMPUTATIONS_H
#define RTT_PASSCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>
#include <world/Field.h>
#include <world/World.hpp>
#include "PositionComputations.h"

namespace rtt::ai::stp::computations {

    class PassComputations {
    public:
        /**
         * Checks if there are given bots within the given tube
         * @param passLine Tube area within to check
         * @param robots Vector of RobotViews which needs to be checked
         * @return True if any of the given robots are inside the given Tube
         */
        static bool pathHasAnyRobots(Line passLine, std::vector<rtt_world::view::RobotView> robots);

        /**
         * Return the position (.first) with the highest score (.second) in the positions vector
         * @param positions std::pair of the Vector2 in first and the given score in second
         * @return Vector2 with highest scoring score.
         */
        static Vector2 determineBestPosForPass(std::vector<gen::ScoredPosition>& positions);
    };
}// namespace rtt::ai::stp::computations
#endif //RTT_PASSCOMPUTATIONS_H
