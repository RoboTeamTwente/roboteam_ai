//
// Created by maxl on 19-03-21.
//

#include "stp/evaluations/position/OpennessEvaluation.h"

#include <cmath>

namespace rtt::ai::stp::evaluation {
uint8_t OpennessEvaluation::metricCheck(std::vector<double>& enemyDistances) noexcept {
    double evalScore = 1.0;
    double maxDist = 1.0;
    for (auto& distance : enemyDistances) {
        if (distance < maxDist) {
            evalScore -= std::pow(-distance / maxDist + 1, 2);
        }
    }
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation