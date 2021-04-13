//
// Created by jessevw on 24.03.20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/Halt.h"

#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"
#include "include/roboteam_ai/stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

    Halt::Halt() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Halt>(role::Halt("halt_0")),
                std::make_unique<role::Halt>(role::Halt("halt_1")),
                std::make_unique<role::Halt>(role::Halt("halt_2")),
                std::make_unique<role::Halt>(role::Halt("halt_3")),
                std::make_unique<role::Halt>(role::Halt("halt_4")),
                std::make_unique<role::Halt>(role::Halt("halt_5")),
                std::make_unique<role::Halt>(role::Halt("halt_6")),
                std::make_unique<role::Halt>(role::Halt("halt_7")),
                std::make_unique<role::Halt>(role::Halt("halt_8")),
                std::make_unique<role::Halt>(role::Halt("halt_9")),
                std::make_unique<role::Halt>(role::Halt("halt_10"))};
    }

    uint8_t Halt::score(PlayEvaluator &playEvaluator) noexcept {
        return playEvaluator.getGlobalEvaluation(GlobalEvaluation::HaltGameState);
    }

    bool Halt::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap Halt::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        const std::string roleName = "halt_";
        for (int i = 0; i <= 10; i++) {
            flagMap.insert({roleName + std::to_string(i), {DealerFlagPriority::REQUIRED, {}}});
        }

        return flagMap;
    }

    void Halt::calculateInfoForRoles() noexcept {}

    void Halt::calculateInfoForScoredRoles(world::World *) noexcept {}

    const char *Halt::getName() { return "Halt"; }
}  // namespace rtt::ai::stp::play
