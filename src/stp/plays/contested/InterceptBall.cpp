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
                                                                                       std::make_unique<role::Formation>(("waller_1")),
                                                                                       std::make_unique<role::Formation>(("waller_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("pass_defender_1")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("pass_defender_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("pass_defender_3")),
                                                                                       std::make_unique<role::BallInterceptor>(role::BallInterceptor("interceptor_1")),
                                                                                       std::make_unique<role::BallInterceptor>(role::BallInterceptor("interceptor_2")),
                                                                                       std::make_unique<role::BallInterceptor>(role::BallInterceptor("interceptor_3"))};
}

uint8_t InterceptBall::score(const rtt::world::Field& field) noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? 255 : 0;
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag interceptorFlag(DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"pass_defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"pass_defender_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"pass_defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"pass_defender_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"interceptor_1", {DealerFlagPriority::REQUIRED, {interceptorFlag}}});
    flagMap.insert({"interceptor_2", {DealerFlagPriority::REQUIRED, {interceptorFlag}}});
    flagMap.insert({"interceptor_3", {DealerFlagPriority::MEDIUM_PRIORITY, {interceptorFlag}}});
    flagMap.insert({"ball_blocker", {DealerFlagPriority::MEDIUM_PRIORITY}});

    return flagMap;
}

void InterceptBall::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    stpInfos["ball_blocker"].setPositionToMoveTo(PositionComputations::getBallBlockPosition(field, world));
    if (stpInfos["ball_blocker"].getRobot())
        stpInfos["ball_blocker"].setAngle((world->getWorld()->getBall()->get()->position - stpInfos["ball_blocker"].getRobot()->get()->getPos()).toAngle());

    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        if (enemy->hasBall()) continue;
        enemyMap.insert({score, enemy->getPos()});
    }

    constexpr auto midfielderNames = std::array{"pass_defender_1", "pass_defender_2", "pass_defender_3", "pass_defender_4"};
    auto activeMidfielderNames = std::vector<std::string>{};
    for (auto name : midfielderNames) {
        if (stpInfos[name].getRobot().has_value()) activeMidfielderNames.emplace_back(name);
    }

    for (int i = 0; i < activeMidfielderNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto& midfielderStpInfo = stpInfos[activeMidfielderNames[i]];
        if (enemyMap.empty()) break;
        midfielderStpInfo.setPositionToDefend(enemyMap.begin()->second);
        midfielderStpInfo.setBlockDistance(BlockDistance::ROBOTRADIUS);
        enemyMap.erase(enemyMap.begin());
    }

}

void InterceptBall::calculateInfoForDefenders() noexcept {
    constexpr auto wallerNames = std::array{"waller_1", "waller_2"};
    auto activeWallerNames = std::vector<std::string>{};
    for (auto name : wallerNames) {
        if (stpInfos[name].getRobot().has_value()) activeWallerNames.emplace_back(name);
    }

    for (int i = 0; i < activeWallerNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto positionToMoveTo = PositionComputations::getWallPosition(i, activeWallerNames.size(), field, world);
        auto& wallerStpInfo = stpInfos[activeWallerNames[i]];

        wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
        wallerStpInfo.setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());

        // If the waller is close to its target, ignore collisions
        constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
        if ((wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
            wallerStpInfo.setShouldAvoidOurRobots(false);
        }
    }
}

void InterceptBall::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

bool InterceptBall::shouldEndPlay() noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT * 0.9;
}

const char* InterceptBall::getName() { return "InterceptBall"; }
}  // namespace rtt::ai::stp::play
