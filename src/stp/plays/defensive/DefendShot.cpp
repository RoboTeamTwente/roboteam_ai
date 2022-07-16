//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendShot.h"

#include <stp/roles/passive/Formation.h>

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"

namespace rtt::ai::stp::play {

DefendShot::DefendShot() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_4")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_5")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_6")),
                                                                                       std::make_unique<role::Harasser>(role::Harasser("harasser")),
                                                                                       std::make_unique<role::Formation>("ball_blocker")};
}

uint8_t DefendShot::score(const rtt::world::Field& field) noexcept {
    if (world->getWorld()->whichRobotHasBall(world::them) != std::nullopt) return 255;
    if (world->getWorld()->whichRobotHasBall(world::us) != std::nullopt) return 0;
    return 0;
}

Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_5", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_6", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"harasser", {DealerFlagPriority::HIGH_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"ball_blocker", {DealerFlagPriority::HIGH_PRIORITY, {notImportant}}});

    return flagMap;
}

void DefendShot::calculateInfoForRoles() noexcept {
    // Expand the defense area by ball_blocker defense threshold
    const auto isBallToClose =
        world->getWorld()->getBall().has_value() && FieldComputations::getDefenseArea(field, true, 0.61, 0).contains(world->getWorld()->getBall()->get()->position);

    if (!isBallToClose) calculateInfoForBlocker();
    calculateInfoForWallers(isBallToClose);
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForHarasser();
}

void DefendShot::calculateInfoForWallers(bool shouldIncludeBallBlocker) noexcept {
    auto wallerNames = std::vector{"waller_1", "waller_2"};
    if (shouldIncludeBallBlocker) wallerNames.push_back("ball_blocker");

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

void DefendShot::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        if (enemy->hasBall()) continue;
        enemyMap.insert({score, enemy->getPos()});
    }

    constexpr auto midfielderNames = std::array{"midfielder_1", "midfielder_2", "midfielder_3", "midfielder_4", "midfielder_5", "midfielder_6"};
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

void DefendShot::calculateInfoForBlocker() noexcept {
    stpInfos["ball_blocker"].setPositionToMoveTo(PositionComputations::getBallBlockPosition(field, world));
    if (stpInfos["ball_blocker"].getRobot())
        stpInfos["ball_blocker"].setAngle((world->getWorld()->getBall()->get()->position - stpInfos["ball_blocker"].getRobot()->get()->getPos()).toAngle());
}

void DefendShot::calculateInfoForHarasser() noexcept {
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    if (!stpInfos["harasser"].getRobot() || !enemyClosestToBall) {
        stpInfos["harasser"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position);
        return;
    }

    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto robotToBallAngle = (ballPos - stpInfos["harasser"].getRobot()->get()->getPos()).toAngle();
    auto ballToEnemyAngle = (enemyClosestToBall->get()->getPos() - ballPos).toAngle();
    auto angleDiff = robotToBallAngle.shortestAngleDiff(ballToEnemyAngle);
    RTT_DEBUG(angleDiff);
    if (angleDiff > M_PI / 3.0) {  // If the enemy is between us and the ball, dont go to the ball directly but further away, to avoid crashing
        auto enemyPos = enemyClosestToBall->get()->getPos();
        auto targetPos = FieldComputations::projectPointToValidPositionOnLine(field, enemyPos + (ballPos - enemyPos).stretchToLength(0.50), enemyPos,
                                                                              enemyPos + (ballPos - enemyPos).stretchToLength(10), AvoidObjects{}, 0.0,
                                                                              control_constants::ROBOT_RADIUS * 2, 0.0);
        RTT_DEBUG(targetPos);
        stpInfos["harasser"].setPositionToMoveTo(targetPos);
        stpInfos["harasser"].setAngle((enemyPos - ballPos).angle());
    } else {
        stpInfos["harasser"].setShouldAvoidTheirRobots(false); // Allow the harasser to get close to the enemy robot by not caring about collisions with enemy robots
        auto harasser = std::find_if(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "harasser"; });
        if (harasser != roles.end() && !harasser->get()->finished() && strcmp(harasser->get()->getCurrentTactic()->getName(), "Formation") == 0) harasser->get()->forceNextTactic();
    }
}

void DefendShot::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

const char* DefendShot::getName() { return "Defend Shot"; }

// If we have the ball we should end doing defendShot
bool DefendShot::shouldEndPlay() noexcept {
    auto robotWithBall = world->getWorld()->whichRobotHasBall(world::both);
    return robotWithBall && robotWithBall->get()->getTeam() == rtt::world::Team::us;
}

}  // namespace rtt::ai::stp::play
