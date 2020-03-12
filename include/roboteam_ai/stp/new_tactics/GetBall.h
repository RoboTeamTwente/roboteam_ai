//
// Created by ratoone on 10-03-20.
//

#ifndef RTT_GETBALL_H
#define RTT_GETBALL_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
class GetBall : protected Tactic {
   protected:
    void onInitialize() noexcept override;

    void onUpdate(Status const &status) noexcept override;

    void onTerminate() noexcept override;

    StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;
};
}  // namespace rtt::ai::stp

#endif  // RTT_GETBALL_H
