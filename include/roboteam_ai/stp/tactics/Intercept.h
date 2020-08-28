//
// Created by timovdk on 3/18/20.
//

#ifndef RTT_INTERCEPT_H
#define RTT_INTERCEPT_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
class Intercept : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    Intercept();

   private:
    /**
     * Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Is this tactic failing during execution (go back to the previous tactic)
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     * This tactic fails if the ball does not move
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Returns true when the robot does not have the ball
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Is this tactic an end tactic?
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;

    /**
     * Calculate the angle between the robot and the ball
     * @param robot Robot
     * @param ball Ball
     * @return Angle between robot and ball
     */
    double calculateAngle(const world::view::RobotView &robot, const world::view::BallView &ball);

    /**
     * Determine the dribbler speed
     * Turn dribbler at full speed when ball is close to robot, otherwise do not dribble
     * @param robot Robot
     * @return Dribbler speed in %
     */
    int determineDribblerSpeed(const world::view::RobotView &robot);
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_INTERCEPT_H
