//
// Created by ratoone on 10-03-20.
//

#ifndef RTT_GETBALL_H
#define RTT_GETBALL_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * This tactic is for getting the ball. It has 3 skills: GoToPos, Rotate, and SetDribbler.
 * It cannot fail, and it's getting reset when there the robot loses the ball. It's not an
 * end tactic, therefore it can succeed.
 */
class GetBall : public Tactic {
public:
    GetBall();

protected:
    void onInitialize() noexcept override;

    void onUpdate(Status const &status) noexcept override;

    void onTerminate() noexcept override;

    StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Check base class for usages. The current tactic cannot fail, only reset.
     * @param info
     * @return always false
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * This tactic will be reset when the robot looses the ball
     * @param info
     * @return true if the robot lost the ball
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;
};
}  // namespace rtt::ai::stp

#endif  // RTT_GETBALL_H
