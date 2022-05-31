//
// Created by maxl on 29-03-21.
//

#ifndef RTT_BLOCKINGEVALUATION_H
#define RTT_BLOCKINGEVALUATION_H

#include <cstdint>
#include <vector>

namespace rtt::ai::stp::evaluation {

class BlockingEvaluation {
   public:
    /**
     * Score ability to block robots from position based on enemy positions
     * @param pointDistance distance from ball to point
     * @param enemyDistances enemy distances to ball
     * @param enemyAngles vector of angles between the line from the ball to the point and from the ball to the enemy
     * @return uint8_t score
     */
    [[nodiscard]] static uint8_t metricCheck(double pointDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) noexcept;
};
}  // namespace rtt::ai::stp::evaluation
#endif  // RTT_BLOCKINGEVALUATION_H
