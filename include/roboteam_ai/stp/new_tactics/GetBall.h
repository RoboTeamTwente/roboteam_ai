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

    /**
     * See base class' implementation for details. <br><br>
     * Extra information for this skill is the target position (which will be a point
     * close to the ball between it and the robot, the rotation angle (the robot will
     * face the ball after getting close) and dribbler.
     * @param info tactic info passed from play
     * @return std::optional<SkillInfo> based on the TacticInfo
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

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

    bool isEndTactic() noexcept override;

    /**
     * Checks if this tactic should be forced success
     * It is forced success whenever the corresponding robot has the ball
     * @param info
     * @return whether the tactic is forced success
     */
    bool forceTacticSuccess(const StpInfo &info) noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_GETBALL_H
