//
// Created by rtt-vision on 23-06-20.
//

#ifndef RTT_SHOOTATPOS_H
#define RTT_SHOOTATPOS_H



#include "include/roboteam_ai/stp/Tactic.h"

namespace rtt::ai::stp::tactic {

    class ShootAtPos : public Tactic {
    public:
        ShootAtPos();

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
         * @return std::optional<SkillInfo> based on the TacticInfo
         */
        std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

        /**
         * Calculate the kickforce for the skill
         * @param distance distance to the target
         * @param desiredBallSpeedType type of the kick
         * @return the speed the kicker needs to kick at
         */
        double determineKickForce(double distance, KickChipType desiredBallSpeedType) noexcept;

        bool isEndTactic() noexcept override;

        bool isTacticFailing(const StpInfo &info) noexcept override;

        bool shouldTacticReset(const StpInfo &info) noexcept override;

        /**
         * Gets the tactic name
         */
        const char *getName() override;

        std::optional<StpInfo> calculateInfoForKick(const StpInfo &info) noexcept;

        std::optional<StpInfo> calculateInfoForChip(const StpInfo &info) noexcept;

        double determineChipForce(double distance, KickChipType desiredBallSpeedType) noexcept;
    };
}  // namespace rtt::ai::stp::tactic


#endif //RTT_SHOOTATPOS_H
