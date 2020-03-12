//
// Created by jessevw on 12.03.20.
//

#ifndef RTT_BLOCKROBOT_H
#define RTT_BLOCKROBOT_H

#include <include/roboteam_ai/stp/Tactic.h>

namespace rtt::ai::stp::tactic {
    class BlockRobot : Tactic {
    public:
        BlockRobot();

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

        double calculateAngle(const world_new::view::RobotView enemy, Vector2 targetLocation);


        Vector2 calculateMoveTarget(BlockDistance blockDistance, const world_new::view::RobotView enemy,
                            Vector2 targetLocation);
    };
}


#endif //RTT_BLOCKROBOT_H
