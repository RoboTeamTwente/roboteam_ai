//
// Created by jessevw on 17.03.20.
//

#include <include/roboteam_ai/stp/new_roles/Halt.h>
#include "stp/new_plays/Pass.h"

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallMovesSlowInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"
#include "stp/new_roles/TestRole.h"
#include "roboteam_utils/Tube.h"
#include "roboteam_utils/Grid.h"


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
        std::make_unique<role::Halt>(role::Halt("defender1")),      std::make_unique<role::Halt>(role::Halt("test_role_3")),
        std::make_unique<role::Halt>(role::Halt("test_role_4")),    std::make_unique<role::Halt>(role::Halt("test_role_5")),
        std::make_unique<role::Halt>(role::Halt("test_role_6")),    std::make_unique<role::Halt>(role::Halt("test_role_7")),
        std::make_unique<role::Halt>(role::Halt("test_role_8")),    std::make_unique<role::Halt>(role::Halt("test_role_9")),
        std::make_unique<role::Halt>(role::Halt("test_role_10"))};
}

uint8_t Pass::score(world_new::World* world) noexcept { return 100; }

Dealer::FlagMap Pass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"pass_receiver", {not_important}});
    flagMap.insert({"defender1", {not_important}});
    flagMap.insert({"test_role_3", {not_important}});
    flagMap.insert({"test_role_4", {not_important}});
    flagMap.insert({"test_role_5", {not_important}});
    flagMap.insert({"test_role_6", {not_important}});
    flagMap.insert({"test_role_7", {not_important}});
    flagMap.insert({"test_role_8", {not_important}});
    flagMap.insert({"test_role_9", {not_important}});
    flagMap.insert({"test_role_10", {not_important}});

    return flagMap;
}

void Pass::calculateInfoForRoles() noexcept {
    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 1;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    // TODO: compute the passing position
    const Vector2 passingPosition = calculatePassLocation();

    // Receiver
    stpInfos["pass_receiver"].setPositionToMoveTo(passingPosition);

    // Passer
    stpInfos["passer"].setPositionToShootAt(passingPosition);
    stpInfos["passer"].setKickChipType(PASS);

    // Defenders
    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }
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

bool Pass::shouldRoleSkipEndTactic() { return false; }

const char* Pass::getName() { return "Pass"; }

    Vector2 Pass::calculatePassLocation() {
        auto ourBots = world->getWorld()->getUs();
        auto theirBots = world->getWorld()->getThem();

        double fieldWidth = field.getFieldWidth();
        double offSetX = 0.3 * fieldWidth; // start looking for suitable positions to move to at 30% of the field width
        double offSetY = 0;
        double regionWidth = 3;
        double regionHeight = 3;
        auto numStepsX = 10;
        auto numStepsY = 10;

        double bestScore = 0;
        Vector2 bestPosition{};

        Grid grid = Grid(offSetX, offSetY, regionWidth, regionHeight, numStepsX, numStepsY);
        for (const auto& nestedPoints : grid.getPoints()) {
            for (const auto& trial : nestedPoints) {
                auto w = world->getWorld().value();
                auto percentage = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, trial, w);
                auto fieldDiagonalLength = sqrt(pow(fieldWidth, 2.0) + pow(field.getFieldLength(), 2.0));

                // Normalize distance, and then subtract 1
                // This inverts the score, so if the distance is really large,
                // the score for the distance will be close to 0
                auto goalDistance = 1 - FieldComputations::getDistanceToGoal(field, false, trial) / fieldDiagonalLength;
                goalDistance *= 100;
                auto pointScore = goalDistance + percentage;

                // Robot is never allowed to stand in their defense area, so exclude any points in the grid that are in it
                if (pointScore > bestScore && !FieldComputations::pointIsInDefenseArea(field, trial, false)) {
                    bestScore = pointScore;
                    bestPosition = trial;
                }
            }
        }
        return bestPosition;
    }

}  // namespace rtt::ai::stp::play
