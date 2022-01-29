//
// Created by Alexander on 29-01-2022
//

#ifndef RTT_BALLINOURDEFENSEAREAANDSTILL_H
#define RTT_BALLINOURDEFENSEAREAANDSTILL_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class BallInOurDefenseAreaAndStillGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallInOurDefenseAreaAndStillGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLINOURDEFENSEAREAANDSTILL_H
