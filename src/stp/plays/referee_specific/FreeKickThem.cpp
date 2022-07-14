//
// Created by jordi on 07-05-20.
//

#include "stp/plays/referee_specific/FreeKickThem.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {
const char* FreeKickThem::getName() { return "Free Kick Them"; }

FreeKickThem::FreeKickThem() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_helper_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_helper_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
        std::make_unique<role::Formation>(role::Formation("harasser")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_helper_3")),
        std::make_unique<role::Formation>(role::Formation("offender_1")),
    };
}

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"harasser", {DealerFlagPriority::HIGH_PRIORITY, {closestToBallFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_3", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {closeToTheirGoalFlag}}});

    return flagMap;  // DONT TOUCH.
}

uint8_t FreeKickThem::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::FreeKickThemGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}
void FreeKickThem::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForHarassers();
    calculateInfoForOffenders();
}

void FreeKickThem::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setBlockDistance(BlockDistance::PARTWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setBlockDistance(BlockDistance::PARTWAY);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    auto enemyClosestToOurGoalOne = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalOne && enemyRobot->getId() == enemyClosestToOurGoalOne.value()->getId(); });

    auto enemyClosestToOurGoalTwo = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalTwo && enemyRobot->getId() == enemyClosestToOurGoalTwo.value()->getId(); });

    stpInfos["defender_helper_1"].setPositionToDefend(enemyClosestToOurGoalOne->get()->getPos());
    stpInfos["defender_helper_1"].setBlockDistance(BlockDistance::ROBOTRADIUS);

    stpInfos["defender_helper_2"].setPositionToDefend(enemyClosestToOurGoalTwo->get()->getPos());
    stpInfos["defender_helper_2"].setBlockDistance(BlockDistance::ROBOTRADIUS);

    stpInfos["defender_helper_3"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_helper_3"].setBlockDistance(BlockDistance::HALFWAY);

    std::map<double, Vector2> enemyMap;

    // TODO: figure out better scoring
    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getTotalGoalAngle(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 3; i++) {
        if (enemyMap.empty()) {
            break;
        }
        stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.rbegin()->second);
        stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::ROBOTRADIUS);
        enemyMap.erase(prev(enemyMap.end()));
    }
}

void FreeKickThem::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

void FreeKickThem::calculateInfoForHarassers() noexcept {
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto enemyPos = world->getWorld()->getRobotClosestToBall(world::Team::them)->get()->getPos();
    auto targetPos = ballPos + (ballPos - enemyPos).stretchToLength(0.6);
    double angle = Vector2(enemyPos-ballPos).toAngle();

    stpInfos["harasser"].setAngle(angle);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
}

void FreeKickThem::calculateInfoForOffenders() noexcept {
    stpInfos["offender_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
    if (world->getWorld()->getBall().value()->position.y > 0) {
        stpInfos["offender_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
    } else {
        stpInfos["offender_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    }
}

}  // namespace rtt::ai::stp::play
