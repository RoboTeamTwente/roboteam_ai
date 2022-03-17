//
// Created by maxl on 19-03-21.
//

#ifndef RTT_OPENNESSEVALUATION_H
#define RTT_OPENNESSEVALUATION_H

#include <cstdint>
#include <vector>

namespace rtt::ai::stp::evaluation {

class OpennessEvaluation {
   public:
    /**
     * Score openness of position based on enemy distances to point (quadratically correlated)
     * @param enemyDistances Vector of all enemy distances from the point
     * @return uint8_t score
     */
    [[nodiscard]] static uint8_t metricCheck(std::vector<double>& enemyDistances) noexcept;
};
}  // namespace rtt::ai::stp::evaluation
#endif  // RTT_OPENNESSEVALUATION_H
