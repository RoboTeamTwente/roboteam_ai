//
// Created by jessevw on 17.03.20.
//
#include "stp/new_plays_analysis/PassProblem.h"
#include "stp/new_plays/Pass.h"

#include <stp/invariants/WeHaveBallInvariant.h>

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallMovesSlowInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"

#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Attacker.h"
#include "stp/new_roles/Harasser.h"

namespace rtt::ai::stp::play {

Pass::Pass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());


    keepPlayInvariants.clear();

    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallMovesSlowInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Passer>(role::Passer("passer")), std::make_unique<role::PassReceiver>(role::PassReceiver("pass_receiver")),
        std::make_unique<role::Defender>(role::Defender("defender1")),      std::make_unique<role::Defender>(role::Defender("defender2")),
        std::make_unique<role::Defender>(role::Defender("defender3")),    std::make_unique<role::Harasser>(role::Harasser("midfielder1")),
        std::make_unique<role::Harasser>(role::Harasser("midfielder2")),    std::make_unique<role::Harasser>(role::Harasser("midfielder3")),
        std::make_unique<role::Harasser>(role::Harasser("attacker1")),    std::make_unique<role::Harasser>(role::Harasser("attacker2")),
        std::make_unique<role::Harasser>(role::Harasser("attacker3"))};
    currentPassLocation = {0,0};
    currentPassScore = 1000;
}

uint8_t Pass::score(world_new::World* world) noexcept { return 100; }

Dealer::FlagMap Pass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"pass_receiver", {closeToTheirGoalFlag}});
    flagMap.insert({"defender1", {not_important}});
    flagMap.insert({"defender2", {closeToTheirGoalFlag}});
    flagMap.insert({"defender3", {closeToBallFlag}});
    flagMap.insert({"midfielder1", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"midfielder2", {closeToBallFlag}});
    flagMap.insert({"midfielder3", {closeToTheirGoalFlag}});
    flagMap.insert({"attacker1", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"attacker2", {closeToBallFlag}});
    flagMap.insert({"attacker3", {closeToTheirGoalFlag}});

    return flagMap;
}

void Pass::calculateInfoForRoles() noexcept {
    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 1;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    const Vector2 passingPosition = Vector2{archipelago->get_champions_x()[0][0], archipelago->get_champions_x()[0][1]};

    // Calculate receiver info
    if (stpInfos.find("pass_receiver") != stpInfos.end()) {
        auto pos = archipelago->get_champions_x();
        auto candidatePosition = Vector2(pos[0][0],pos[0][1]);
        auto chosenPosition = compareNewLocationToCurrentLocation(currentPassLocation, candidatePosition);
        currentPassLocation = chosenPosition.first;
        currentPassScore = chosenPosition.second;
        stpInfos["pass_receiver"].setPositionToMoveTo(chosenPosition.first);
    }

    // Calculate Passer info
    if (stpInfos.find("passer") != stpInfos.end()) {
        stpInfos["passer"].setPositionToShootAt(passingPosition);
        stpInfos["passer"].setKickChipType(PASS);
    }

    // Defenders
    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }
    std::vector<Vector2> defendPoints = {Vector2(0,0.5), Vector2(0,-0.5)};
    stpInfos["defender1"].setEnemyRobot(world->getWorld()->getRobotClosestToBall());
    stpInfos["defender2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(defendPoints[0]));
    stpInfos["defender3"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(defendPoints[1]));
    stpInfos["defender1"].setBlockDistance(BlockDistance::HALFWAY);
    stpInfos["defender2"].setBlockDistance(BlockDistance::HALFWAY);
    stpInfos["defender3"].setBlockDistance(BlockDistance::HALFWAY);
//    stpInfos["attacker1"].setEnemyRobot();
//    stpInfos["attacker2"].setEnemyRobot();
//    stpInfos["attacker3"].setEnemyRobot();
//    stpInfos["midfielder1"].setEnemyRobot();
//    stpInfos["midfielder2"].setEnemyRobot();
//    stpInfos["midfielder3"].setEnemyRobot();

}

std::vector<Vector2> Pass::calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
    std::vector<Vector2> positions = {};

    // 3 robots will defend goal
    for (int i = 0; i < numberOfDefenders; i++) {
        if (i < 3) {
            positions.push_back(world->getField()->getOurGoalCenter());
        } else {
            positions.push_back(enemyRobots[i].get()->getPos());
        }
    }

    return positions;
}
std::pair<Vector2, double>
Pass::compareNewLocationToCurrentLocation(Vector2 currentPosition, Vector2 candidatePosition) {
    const std::vector<double> m = {0,1};
    auto currentPositionScore = PassProblem::cost_function(currentPosition, world_new::World::instance()->getWorld().value(), world->getField().value());
    auto candidatePositionScore = PassProblem::cost_function(candidatePosition, world_new::World::instance()->getWorld().value(), world->getField().value());

    auto cand = std::make_pair(candidatePosition, candidatePositionScore);
    auto curr = std::make_pair(currentPosition, currentPositionScore);

    // Always choose a score that is negative, if possible
    if (currentPositionScore > 0 && !(candidatePositionScore > 0)) {
        return cand;
    }

    // Ratio of scores
    if (candidatePositionScore/currentPositionScore > 1.7) {
        return cand;
    }

    // Proximity of candidate and current locations:
    if ((currentPosition - candidatePosition).length() > 0.1 *  field.getFieldLength()) {
        return curr;
    }

    return cand;
}

bool Pass::shouldRoleSkipEndTactic() { return false; }

const char* Pass::getName() { return "Pass"; }

}  // namespace rtt::ai::stp::play
