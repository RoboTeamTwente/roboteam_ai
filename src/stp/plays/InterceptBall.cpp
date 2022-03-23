//
// Created by sarah on 02-03-22.
//

#include "stp/plays/InterceptBall.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Harasser.h"
#include "stp/roles/passive/Halt.h"
#include "stp/roles/active/Intercepter.h"

namespace rtt::ai::stp::play {

InterceptBall::InterceptBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallIsFree);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallIsFree);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Halt>(role::Halt("halt_1")),
        std::make_unique<role::Halt>(role::Halt("halt_2")),
        std::make_unique<role::Intercepter>(role::Intercepter("intercepter")),
    };
}

uint8_t InterceptBall::score(PlayEvaluator &playEvaluator) noexcept {
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"intercepter", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
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
