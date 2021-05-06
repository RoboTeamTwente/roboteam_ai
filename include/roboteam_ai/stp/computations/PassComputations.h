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
    };
}// namespace rtt::ai::stp::computations
#endif //RTT_PASSCOMPUTATIONS_H
