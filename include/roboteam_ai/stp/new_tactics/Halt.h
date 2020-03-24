//
// Created by jessevw on 24.03.20.
//

#ifndef RTT_HALT_TACTIC_H
#define RTT_HALT_TACTIC_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * This tactic is for getting the ball. It has 3 skills: GoToPos, Rotate, and SetDribbler.
 * It cannot fail, and it's getting reset when there the robot loses the ball. It's not an
 * end tactic, therefore it can succeed.
 */
    class Halt : public Tactic {
    public:
        Halt();

    protected:
        void onInitialize() noexcept override;

        void onUpdate(Status const &status) noexcept override;

        void onTerminate() noexcept override;

        /**
         * See base class' implementation for details. <br><br>
         * Extra information for this skill is the target position and rotation angle
         * @param info tactic info passed from play
         * @return the modified tactic info with the new data
         */
        StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

        /**
         * Check base class for usages. The current tactic cannot fail
         * @param info
         * @return always false
         */
        bool isTacticFailing(const StpInfo &info) noexcept override;

        /**
         * This tactic cannot be reset
         * @param info
         * @return true if the robot lost the ball
         */
        bool shouldTacticReset(const StpInfo &info) noexcept override;

        /**
         * This tactic is an end tactic
         * @return true
         */
        bool isEndTactic() noexcept override;
    };
}  // namespace rtt::ai::stp::tactic


#endif //RTT_HALT_TACTIC_H
