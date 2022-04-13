//
// Created by sarah on 23-03-22.
//

#include "stp/plays/offensive/InterceptBall.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Harasser.h"
#include "stp/roles/passive/Halt.h"
#include "stp/roles/active/Intercepter.h"
#include "stp/computations/GoalComputations.h"

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
        std::make_unique<role::Intercepter>(role::Intercepter("intercepter")),
        std::make_unique<role::Halt>(role::Halt("halt_0")),
        std::make_unique<role::Halt>(role::Halt("halt_1")),
        std::make_unique<role::Halt>(role::Halt("halt_2")),
        std::make_unique<role::Halt>(role::Halt("halt_3")),
        std::make_unique<role::Halt>(role::Halt("halt_4")),
        std::make_unique<role::Halt>(role::Halt("halt_5")),
        std::make_unique<role::Halt>(role::Halt("halt_6")),
        std::make_unique<role::Halt>(role::Halt("halt_7")),
        std::make_unique<role::Halt>(role::Halt("halt_8"))
    };
}

uint8_t InterceptBall::score(PlayEvaluator &playEvaluator) noexcept {
    return 0;
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"intercepter", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void InterceptBall::calculateInfoForRoles() noexcept {

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// TODO let keeper shot at a good position
    stpInfos["keeper"].setPositionToShootAt(Vector2(0,0));

    //auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["intercepter"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos()+control_constants::ROBOT_RADIUS);

}

void InterceptBall::calculateInfoForDefenders() noexcept {
}

void InterceptBall::calculateInfoForHarassers() noexcept {
}

void InterceptBall::calculateInfoForKeeper() noexcept {
}

const char *InterceptBall::getName() { return "Intercept Ball"; }

}  // namespace rtt::ai::stp::play