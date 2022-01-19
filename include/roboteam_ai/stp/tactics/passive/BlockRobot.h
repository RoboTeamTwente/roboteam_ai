//
// Created by jessevw on 12.03.20.
//

#ifndef RTT_BLOCKROBOT_H
#define RTT_BLOCKROBOT_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class BlockRobot : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    BlockRobot();

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
     * This tactic can never fail, so always returns false
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Returns true when the robot is further away from the target position than some error margin
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
     * Find the desired angle for the robot to block the target
     * @param enemy the robot that is being blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position,
     * or our goal
     * @return desired angle for robot to block target
     */
    double calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation);

    /**
     * Find location for robot to move to to block the target
     * @param blockDistance how close the robot should be to the enemy robot.
     * @param enemy the enemy robot to be blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position,
     * or our goal
     * @return the desired position to block the target.
     */
    Vector2 calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation, double enemyDistance);
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_BLOCKROBOT_H
