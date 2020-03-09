//
// Created by roboteam on 9/3/20.
//

#ifndef RTT_TESTTACTIC_H
#define RTT_TESTTACTIC_H

#include "include/roboteam_ai/stp/Tactic.h"
namespace rtt::ai::stp {

class TestTactic : public Tactic {
        void onInitialize() noexcept override;
        Status onUpdate(SkillInfo const& info) noexcept override;
        void onTerminate() noexcept override;
};

} // namespace rtt::ai::stp

#endif //RTT_TESTTACTIC_H
