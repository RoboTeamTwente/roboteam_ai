//
// Created by maxl on 19-03-21.
//

#include "stp/evaluations/position/LineOfSightEvaluation.h"

#include <cmath>

#include "stp/constants/ControlConstants.h"
namespace rtt::ai::stp::evaluation {

uint8_t LineOfSightEvaluation::metricCheck(double pDist, std::vector<double>& eDists, std::vector<double>& eAngles) noexcept {
    /**             _-                                 \
     *           _-           3               3         \
     *        _-  \                  3                   \        The line of sight score is evaluated as follows:
     *     _-   1  \     3               _________________
     *  _-         |__________----------     2           |     1. There is a robot close to the ball and within 30 deg of the ball-to-target line => score = 0
     * B=====1=====|=========2===========================Target     2. There is a robot within 5 deg of the ball-to-target line => score = 0
     *  -_         |----------__________     2           |     3. There is a robot within 30 deg of the ball-to-target line and far away from the ball
     *     -_   1 /                    -----------------|         => score is based on angle of enemy to line (quadratic, with score = 0 at 5 deg, score = 1 at 30 deg)
     *        -_ /    3               3                /
     *           -_           3                     3 /
     *              -_                               /
     */

    double evalScore = 1;           // Default score (= perfect line of sight)
    double innerAngle = M_PI / 36;  // 5 degrees
    double outerAngle = M_PI / 6;   // 30 degrees
    for (int i = 0; i < eAngles.size(); i++) {
        if (eDists[i] < pDist && eAngles[i] < outerAngle) {
            if (eDists[i] < control_constants::DISTANCE_TO_ROBOT_FAR || eAngles[i] < innerAngle) {
                return 0;
            } else {
                evalScore -= std::pow((1 / (outerAngle - innerAngle)) * (outerAngle - eAngles[i]), 2);
            }
        }
    }
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation