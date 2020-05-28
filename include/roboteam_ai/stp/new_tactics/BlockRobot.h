//
// Created by jessevw on 12.03.20.
//

#ifndef RTT_BLOCKROBOT_H
#define RTT_BLOCKROBOT_H

#include <include/roboteam_ai/stp/Tactic.h>
#include <include/roboteam_ai/utilities/Constants.h>

namespace rtt::ai::stp::tactic {

class BlockRobot : public Tactic {
   public:
    BlockRobot();

   private:
    // TODO: TUNE make this a sensible margin
    const double errorMargin = 1 * Constants::ROBOT_RADIUS();
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

    /**
     * Find the desired angle for the robot to block the target
     * @param enemy the robot that is being blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position,
     * or our goal
     * @return desired angle for robot to block target
     */
    double calculateAngle(const world_new::view::RobotView enemy, const Vector2& targetLocation);

    /**
     * Find location for robot to move to to block the target
     * @param blockDistance how close the robot should be to the enemy robot.
     * @param enemy the enemy robot to be blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position,
     * or our goal
     * @return the desired position to block the target.
     */
    Vector2 calculateDesiredRobotPosition(BlockDistance blockDistance, const world_new::view::RobotView enemy, const Vector2& targetLocation);

    bool isTacticFailing(const StpInfo &info) noexcept override;
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_BLOCKROBOT_H
