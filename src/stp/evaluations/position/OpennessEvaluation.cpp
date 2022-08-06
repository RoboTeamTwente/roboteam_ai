//
// Created by maxl on 19-03-21.
//

#include "stp/evaluations/position/OpennessEvaluation.h"

#include <cmath>
#include <algorithm>

namespace rtt::ai::stp::evaluation {
uint8_t OpennessEvaluation::metricCheck(std::vector<double>& enemyDistances, double radius) noexcept {
    double evalScore = 1.0;
    for (auto& distance : enemyDistances) {
        if (distance < radius) {
            evalScore -= std::pow(-distance / radius + 1, 2);
        }
    }
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation