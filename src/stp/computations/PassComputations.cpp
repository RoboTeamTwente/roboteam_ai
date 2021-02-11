//
// Created by maxl on 11-02-21.
//

#include <stp/StpInfo.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>
#include "roboteam_utils/Tube.h"
#include <roboteam_utils/Grid.h>

namespace rtt::ai::stp::computations {
    bool PassComputations::pathHasAnyRobots(Line passLine, std::vector <rtt_world::view::RobotView> robots) {
        Tube passTube = Tube(passLine.v1, passLine.v2 ,control_constants::ROBOT_RADIUS);
        if (std::any_of(robots.begin(), robots.end(),
                            [&](const auto &bot) { return passTube.contains(bot->getPos()); })) {
                return true;
            }
            return false;
        }

    Vector2 PassComputations::determineBestPosForPass(const std::vector <std::pair < Vector2,double>>& positions ) {
        double bestScore = 0.0;
        Vector2 bestPosition{};
        for (std::pair <Vector2,double> pos : positions){
            if (pos.second > bestScore) bestPosition = pos.first;
        }
        return bestPosition;
    }
} //namespace rtt::ai::stp::computations