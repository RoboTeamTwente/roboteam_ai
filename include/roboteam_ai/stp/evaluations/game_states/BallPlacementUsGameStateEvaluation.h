//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLPLACEMENTUSGAMESTATEEVALUATION_H
#define RTT_BALLPLACEMENTUSGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class BallPlacementUsGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::BallPlacementUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLPLACEMENTUSGAMESTATEEVALUATION_H
