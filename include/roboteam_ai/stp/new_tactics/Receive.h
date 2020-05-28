//
// Created by jordi on 13-03-20.
//

#ifndef RTT_RECEIVE_H
#define RTT_RECEIVE_H

#include "include/roboteam_ai/stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class Receive : public Tactic {
   public:
    Receive();

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
     * Calculate info for the skills
     * @param info Info passed by the role
     * @return Info for the skills
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Tactic fails if targetType is not a receiveTarget
     * @param info Info
     * @return True if targetType is not a receiveTarget
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Reset tactic when robot position is not close enough to the target position for receiving
     * @param info Info
     * @return True if robot position is not close enough to the target position
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Calculate the angle between the robot and the ball
     * @param robot Robot
     * @param ball Ball
     * @return Angle between robot and ball
     */
    double calculateAngle(const world_new::view::RobotView &robot, const world_new::view::BallView &ball);

    /**
     * Determine the dribbler speed
     * Turn dribbler at full speed when ball is close to robot, otherwise do not dribble
     * @param robot Robot
     * @return Dribbler speed in %
     */
    int determineDribblerSpeed(const world_new::view::RobotView &robot);

    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};

}  // namespace rtt::ai::stp::tactic

#endif  // RTT_RECEIVE_H
