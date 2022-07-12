//
// Created by Alexander on 11-05-2022
//
#include "stp/plays/contested/InterceptBall.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/active/BallInterceptor.h"


namespace rtt::ai::stp::play {

InterceptBall::InterceptBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallIsFree);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::BallIsFree);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_2")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_3")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_4")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_4")),
                                                                                       std::make_unique<role::BallInterceptor>(role::BallInterceptor("interceptor_1")),
                                                                                       std::make_unique<role::BallInterceptor>(role::BallInterceptor("interceptor_2"))};
}

uint8_t InterceptBall::score(const rtt::world::Field& field) noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? 255 : 0;
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag interceptorFlag(DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY}});
    flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY}});
    flagMap.insert({"waller_3", {DealerFlagPriority::MEDIUM_PRIORITY}});
    flagMap.insert({"waller_4", {DealerFlagPriority::MEDIUM_PRIORITY}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"interceptor_1", {DealerFlagPriority::HIGH_PRIORITY, {interceptorFlag}}});
    flagMap.insert({"interceptor_2", {DealerFlagPriority::HIGH_PRIORITY, {interceptorFlag}}});

    return flagMap;
}

void InterceptBall::calculateInfoForRoles() noexcept {
    calculateInfoForWallers();
    calculateInfoForDefenders();
    calculateInfoForKeeper();
}

void InterceptBall::calculateInfoForWallers() noexcept {
    stpInfos["waller_1"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_2"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_3"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_4"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());

    stpInfos["waller_1"].setPositionToMoveTo(PositionComputations::getWallPosition(0, 4, field, world));
    stpInfos["waller_2"].setPositionToMoveTo(PositionComputations::getWallPosition(1, 4, field, world));
    stpInfos["waller_3"].setPositionToMoveTo(PositionComputations::getWallPosition(2, 4, field, world));
    stpInfos["waller_4"].setPositionToMoveTo(PositionComputations::getWallPosition(3, 4, field, world));
}

void InterceptBall::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 4; i++) {
        if (!enemyMap.empty()) {
            stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.begin()->second);
            stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::ROBOTRADIUS);
            enemyMap.erase(enemyMap.begin());
        } else {
            break;
        }
    }
}

void InterceptBall::calculateInfoForInterceptors() noexcept {

}

void InterceptBall::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
}

bool InterceptBall::shouldEndPlay() noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT * 0.9;
}

const char* InterceptBall::getName() { return "InterceptBall"; }
}  // namespace rtt::ai::stp::play
