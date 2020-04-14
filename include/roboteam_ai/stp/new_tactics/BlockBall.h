//
// Created by jordi on 08-04-20.
//

#ifndef RTT_BLOCKBALL_H
#define RTT_BLOCKBALL_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class BlockBall : public Tactic {
public:
    BlockBall();

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
    StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

    bool isTacticFailing(const StpInfo &info) noexcept override;

    bool shouldTacticReset(const StpInfo &info) noexcept override;

    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;

    /**
     * Calculates the position for the keeper
     * @param ball Ball
     * @param field Field
     * @return Target position for the keeper
     */
    Vector2 calculateTargetPosition(world_new::view::BallView ball, world::Field field) noexcept;
};

} // namespace rtt::ai::stp::tactic

#endif //RTT_BLOCKBALL_H
