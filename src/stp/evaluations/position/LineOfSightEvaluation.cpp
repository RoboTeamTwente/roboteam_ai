//
// Created by maxl on 19-03-21.
//

#include "stp/evaluations/position/LineOfSightEvaluation.h"

#include <cmath>
#include <algorithm>

#include "stp/constants/ControlConstants.h"
namespace rtt::ai::stp::evaluation {

uint8_t LineOfSightEvaluation::metricCheck(double pDist, std::vector<double>& eDists, std::vector<double>& eAngles) noexcept {
    /**             _-                                \
     *           _-                                    \
     *        _-                                        \        The line of sight score is evaluated as follows:
     *     _-                                            \
     *  _-                                               |
     * B=================================================Target   If there is an enemy on the line from the ball to the target (angle=0),
     *  -_                                               |         the LoS score is 0.
     *     -_                                           |         At angles > 0, the score slowly scales up to 1 (which is at 30 deg)
     *        -_                                       /
     *           -_                                   /
     *              -_                               /
     */

    double evalScore = 1;                    // Default score (= perfect line of sight)
    constexpr double outerAngle = M_PI / 6;  // 30 degrees- enemies outside this angle are not considered
    for (size_t i = 0; i < eAngles.size(); i++) {
        if (eAngles[i] < outerAngle) {
            // This scales from 0 (at 4 radii in front of the enemy) to 1 (at 4 radii behind the enemy). This smoothens the transitions around robots
            auto distFadeFactor = std::clamp((-1.0 / (8 * control_constants::ROBOT_RADIUS) * (eDists[i] - pDist)) + 0.5, 0.0, 1.0);
            evalScore -= std::pow((1 / (outerAngle)) * (outerAngle - eAngles[i]), 2) * distFadeFactor;
        }
    }
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation