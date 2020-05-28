//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/AttackingPass.h"

#include <stp/new_roles/Halt.h>
#include <stp/new_roles/Keeper.h>

#include "roboteam_utils/Grid.h"
#include "roboteam_utils/Tube.h"
#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallMovesSlowInvariant.h"
#include "stp/invariants/NoGoalVisionFromBallInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"
#include "stp/new_roles/TestRole.h"

namespace rtt::ai::stp::play {

AttackingPass::AttackingPass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::NoGoalVisionFromBallInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallMovesSlowInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::NoGoalVisionFromBallInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Passer>(role::Passer("passer")),  std::make_unique<role::PassReceiver>(role::PassReceiver("pass_receiver")),
        std::make_unique<role::Halt>(role::Halt("defender1")),   std::make_unique<role::Halt>(role::Halt("test_role_3")),
        std::make_unique<role::Halt>(role::Halt("test_role_4")), std::make_unique<role::Halt>(role::Halt("test_role_5")),
        std::make_unique<role::Halt>(role::Halt("test_role_6")), std::make_unique<role::Halt>(role::Halt("test_role_7")),
        std::make_unique<role::Halt>(role::Halt("test_role_8")), std::make_unique<role::Halt>(role::Halt("test_role_9"))};
}

uint8_t AttackingPass::score(world_new::World* world) noexcept { return 100; }

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"passer", {passerFlag}});
    flagMap.insert({"pass_receiver", {receiverFlag}});
    flagMap.insert({"defender1", {not_important}});
    flagMap.insert({"test_role_3", {not_important}});
    flagMap.insert({"test_role_4", {not_important}});
    flagMap.insert({"test_role_5", {not_important}});
    flagMap.insert({"test_role_6", {not_important}});
    flagMap.insert({"test_role_7", {not_important}});
    flagMap.insert({"test_role_8", {not_important}});
    flagMap.insert({"test_role_9", {not_important}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

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

std::vector<Vector2> AttackingPass::calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
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

bool AttackingPass::shouldRoleSkipEndTactic() { return false; }

const char* AttackingPass::getName() { return "AttackingPass"; }

Vector2 AttackingPass::calculatePassLocation() {
    auto ourBots = world->getWorld()->getUs();
    auto theirBots = world->getWorld()->getThem();

    auto fieldWidth = field.getFieldWidth();
    auto fieldLength = field.getFieldLength();

    double offSetX = 0.3 * fieldWidth;  // start looking for suitable positions to move to at 30% of the field width
    double offSetY = -3;
    double regionWidth = 3;
    double regionHeight = 6;
    auto numStepsX = 20;
    auto numStepsY = 20;

    double bestScore = 0;
    Vector2 bestPosition{};

    auto w = world->getWorld().value();

    // Make a grid with all potentially good points
    Grid grid = Grid(offSetX, offSetY, regionWidth, regionHeight, numStepsX, numStepsY);
    for (const auto& nestedPoints : grid.getPoints()) {
        for (const auto& trial : nestedPoints) {
            // Make sure we only check valid points
            if (!FieldComputations::pointIsInDefenseArea(field, trial, false)) {
                // Check goal visibility from  a point
                auto visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, trial, w) / 100;

                // Normalize distance, and then subtract 1
                // This inverts the score, so if the distance is really large,
                // the score for the distance will be close to 0
                auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);
                auto goalDistance = 1 - (FieldComputations::getDistanceToGoal(field, false, trial) / fieldDiagonalLength);

                // Make sure the angle to shoot at the goal with is okay
                auto trialToGoalAngle = 1 - fabs((field.getTheirGoalCenter() - trial).angle()) / M_PI_2;

                // Make sure the ball can reach the target
                auto canReachTarget{1.0};
                auto passLine = Tube(w->getBall()->get()->getPos(), trial, control_constants::ROBOT_CLOSE_TO_POINT);
                auto enemyBots = w.getThem();
                if (std::any_of(enemyBots.begin(), enemyBots.end(), [&](const auto& bot) { return passLine.contains(bot->getPos()); })) {
                    canReachTarget = 0.0;
                }

                // Search closest bot to this point and get that distance
                auto theirClosestBot = w.getRobotClosestToPoint(trial, world_new::Team::them);
                auto theirClosestBotDistance = theirClosestBot->getPos().dist(trial) / fieldDiagonalLength;

                // Calculate total score for this point
                auto pointScore = (goalDistance + visibility + trialToGoalAngle) * (0.5 * theirClosestBotDistance * canReachTarget);

                // Check for best score
                if (pointScore > bestScore) {
                    bestScore = pointScore;
                    bestPosition = trial;
                }
            }
        }
    }
    return bestPosition;
}

}  // namespace rtt::ai::stp::play
