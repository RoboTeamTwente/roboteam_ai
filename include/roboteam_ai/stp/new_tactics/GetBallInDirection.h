//
// Created by jordi on 06-04-20.
//

#ifndef RTT_GETBALLINDIRECTION_H
#define RTT_GETBALLINDIRECTION_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * This tactic is for getting the ball from a certain direction. It has 3 skills: GoToPos, Rotate, and SetDribbler.
 * It fails when there is no target to point towards and does not reset.
 * It's not an end tactic, therefore it can succeed.
 */
    class GetBallInDirection : public Tactic {
    public:
        GetBallInDirection();

    protected:
        void onInitialize() noexcept override;

        void onUpdate(Status const &status) noexcept override;

        void onTerminate() noexcept override;

        /**
         * See base class' implementation for details. <br><br>
         * Extra information for this skill is the position behind the ball, rotation angle and dribbler speed.
         * @param info tactic info passed from play
         * @return the modified tactic info with the new data
         */
        StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

        /**
         * Check base class for usages. The current tactic fails if there is no target position to aim
         * @param info
         * @return True when there is no positionToShootAt
         */
        bool isTacticFailing(const StpInfo &info) noexcept override;

        /**
         * This tactic does not reset
         * @param info
         * @return Always false
         */
        bool shouldTacticReset(const StpInfo &info) noexcept override;

        /**
         * This tactic is not an end tactic
         * @return Always false
         */
        bool isEndTactic() noexcept override;

        /**
         * Distance between the robot and the ball that is used for the position behind the ball
         * and when the dribbler is turned on
         */
        double ballDistanceMargin = 3 * Constants::ROBOT_RADIUS();

        /**
         * Gets the tactic name
         */
        const char *getName() override;
    };
}  // namespace rtt::ai::stp::tactic

#endif //RTT_GETBALLINDIRECTION_H
