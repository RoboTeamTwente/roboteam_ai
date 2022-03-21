//
// Created by maxl on 23-03-21.
//

#include "stp/evaluations/position/GoalShotEvaluation.h"

#include <algorithm>  // For clamp
#include <cmath>

namespace rtt::ai::stp::evaluation {
uint8_t GoalShotEvaluation::metricCheck(double vis, double goalAngle) noexcept {
    auto evalScore = std::pow(goalAngle, 1.0 / 2.0) * vis;
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation