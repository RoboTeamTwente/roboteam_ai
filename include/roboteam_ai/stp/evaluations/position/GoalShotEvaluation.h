//
// Created by maxl on 23-03-21.
//

#ifndef RTT_GOALSHOTEVALUATION_H
#define RTT_GOALSHOTEVALUATION_H

#include <cstdint>

namespace rtt::ai::stp::evaluation {

class GoalShotEvaluation {
   public:
    /**
     * Score ability to make a goal from position based on how much of the goal is visible and the angular size of the goal
     * @param goalVisibility % visibility of the goal (taking in account other robots)
     * @param goalAngle angular size of the goal (relative to the goal angle from the penalty point)
     * @return uint8_t score
     */
    [[nodiscard]] static uint8_t metricCheck(double goalVisibility, double goalAngle) noexcept;
};
}  // namespace rtt::ai::stp::evaluation
#endif  // RTT_GOALSHOTEVALUATION_H
