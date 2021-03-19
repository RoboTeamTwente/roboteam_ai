//
// Created by maxl on 11-02-21.
//

#include <stp/StpInfo.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>
#include "roboteam_utils/Tube.h"
#include <roboteam_utils/Grid.h>

namespace rtt::ai::stp::computations {

    bool PassComputations::pathHasAnyRobots(Line passLine, std::vector<rtt_world::view::RobotView> robots) {
        Tube passTube = Tube(passLine.v1, passLine.v2, control_constants::ROBOT_RADIUS);
        return std::any_of(robots.begin(), robots.end(),
                           [&](const auto &bot) { return passTube.contains(bot->getPos()); });
    }

//    Vector2 PassComputations::determineBestPosForPass(
//            std::vector<computations::PositionComputations::PositionEvaluation> &positions) {
//        if (!positions.empty()) {
//            return std::max_element(positions.begin(), positions.end(),
//                                    [](PositionComputations::PositionEvaluation &left,
//                                       PositionComputations::PositionEvaluation &right) {
//                                        return left.score < right.score;
//                                    })->position;
//        } else {
//            RTT_DEBUG("There were no possible locations to pass to.");
//            return {};
//        }
//    }
} //namespace rtt::ai::stp::computations