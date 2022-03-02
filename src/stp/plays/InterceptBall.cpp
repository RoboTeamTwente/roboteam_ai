//
// Created by sarah on 02-03-22.
//

#include "stp/plays/InterceptBall.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Harasser.h"

namespace rtt::ai::stp::play {

InterceptBall::InterceptBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")),
        std::make_unique<role::Harasser>(role::Harasser("harassing_defender")),
    };
}

uint8_t InterceptBall::score(PlayEvaluator &playEvaluator) noexcept {
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;


    return flagMap;
}

void InterceptBall::calculateInfoForRoles() noexcept {
}

void InterceptBall::calculateInfoForDefenders() noexcept {
}

void InterceptBall::calculateInfoForHarassers() noexcept {
}

void InterceptBall::calculateInfoForKeeper() noexcept {
}

const char *InterceptBall::getName() { return "Intercept Ball"; }

}  // namespace rtt::ai::stp::play
