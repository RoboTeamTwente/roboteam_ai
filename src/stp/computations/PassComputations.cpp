//
// Created by maxl on 11-02-21.
//

#include <stp/StpInfo.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>

#include <roboteam_utils/Grid.h>

#include <include/roboteam_ai/world/Field.h>
#include "include/roboteam_ai/world/World.hpp"
#include "include/roboteam_ai/world/views/WorldDataView.hpp"

namespace rtt::ai::stp::computations {

    bool PassComputations::pathHasAnyRobots(Tube passLine, std::vector <rtt_world::view::RobotView> robots) {
            if (std::any_of(robots.begin(), robots.end(),
                            [&](const auto &bot) { return passLine.contains(bot->getPos()); })) {
                return true;
            }
            return false;
        }
} //namespace rtt::ai::stp::computations