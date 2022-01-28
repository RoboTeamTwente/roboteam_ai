//
// Created by maxl on 29-03-21.
//

#include "stp/evaluations/position/BlockingEvaluation.h"

#include <cmath>

namespace rtt::ai::stp::evaluation {
uint8_t BlockingEvaluation::metricCheck(double pointDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) noexcept {
    double evalScore = 0;
    double maxAngle = M_PI / 6;  // 30 degrees, max angle considered for blocking
    for (int i = 0; i < enemyAngles.size(); i++) {
        if (enemyDistances[i] > pointDistance && enemyAngles[i] < maxAngle) {
            evalScore += std::pow((1 / (maxAngle)) * (maxAngle - enemyAngles[i]), 2);
        }
    }
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation