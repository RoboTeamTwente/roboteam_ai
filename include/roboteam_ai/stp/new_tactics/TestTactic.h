//
// Created by roboteam on 9/3/20.
//

#ifndef RTT_TESTTACTIC_H
#define RTT_TESTTACTIC_H

#include "include/roboteam_ai/stp/Tactic.h"

namespace rtt::ai::stp {

class TestTactic : public Tactic {
   public:
    TestTactic();

   private:
    /**
     * On initialization of this tactic, initialize the state machine with skills
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     */
    void onUpdate(Status const &status) noexcept override;

    /**
     * On terminate of this tactic, call terminate on all underlying skills
     */
    void onTerminate() noexcept override;

    /**
     * Calculate the SkillInfo from the TacticInfo
     * @param info info is the TacticInfo passed by the role
     * @return SkillInfo based on the TacticInfo
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    bool isTacticFailing(const StpInfo &info) noexcept override;

    bool shouldTacticReset(const StpInfo &info) noexcept override;

    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};

}  // namespace rtt::ai::stp

#endif  // RTT_TESTTACTIC_H
