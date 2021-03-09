//
// Created by jessevw on 17.03.20.
/// TODO-Max change to fowardPass
//

#include "include/roboteam_ai/stp/plays/offensive/AttackingPass.h"
#include "include/roboteam_ai/stp/computations/PositionComputations.h"
#include "include/roboteam_ai/stp/computations/PassComputations.h"

#include <roboteam_utils/Tube.h>

#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "include/roboteam_ai/stp/roles/passive/Halt.h"
#include "stp/roles/Keeper.h"
#include "include/roboteam_ai/stp/roles/active/PassReceiver.h"
#include "include/roboteam_ai/stp/roles/active/Passer.h"
#include "stp/computations/PositionComputations.h"

namespace rtt::ai::stp::play {

    void AttackingPass::onInitialize() noexcept {
        // Make sure we reset the passerShot flag
        passerShot = false;

        // Make sure we calculate pass positions at least once
        receiverPositionRight = computations::PositionComputations::determineBestLineOfSightPosition(gridRight, field, world);
        receiverPositionLeft = computations::PositionComputations::determineBestLineOfSightPosition(gridLeft, field, world);
        passingPosition = receiverPositionRight.first;
    }

AttackingPass::AttackingPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallCloseToUs);
    startPlayEvaluation.emplace_back(GlobalEvaluation::NoGoalVisionFromBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                       std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_left")),
                                                                                       std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_right")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_8")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_9"))};

    // initialize stpInfos
    stpInfos = std::unordered_map<std::string, StpInfo>{};
    for (auto &role : roles) {
        role->reset();
        auto roleName{role->getName()};
        stpInfos.emplace(roleName, StpInfo{});
    }
}

uint8_t AttackingPass::score(PlayEvaluator *playEvaluator) noexcept {
    calculateInfoForScoredRoles(playEvaluator->getWorld());
    scoring = {std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::BallClosestToUs), 2),
               std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::GoalVisionFromBall), -1),
               std::make_pair(std::max({stpInfos["receiver_left"].getRoleScore().value(),stpInfos["receiver_right"].getRoleScore().value()}),1)};
    return (lastScore = calculateScore(scoring)).value();
    }

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER,{}}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED,{passerFlag}}});
    flagMap.insert({"receiver_left", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
    flagMap.insert({"receiver_right", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForScoredRoles(world::World* world) noexcept {
    rtt::world::Field field = world->getField().value();
    /// Recalculate pass positions if we did not shoot yet
    /// For the receive locations, divide the field up into grids where the passers should stand,
    /// and find the best locations in those grids
    receiverPositionRight = computations::PositionComputations::determineBestGoalShotLocation(gridRight, field, world);
    receiverPositionLeft = computations::PositionComputations::determineBestGoalShotLocation(gridLeft, field, world);

    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.first);
    stpInfos["receiver_left"].setRoleScore(receiverPositionLeft.second);
    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.first);
    stpInfos["receiver_right"].setRoleScore(receiverPositionRight.second);
    }

void AttackingPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();
    calculateInfoForScoredRoles(world);
    /// Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    /// Passer and receivers
    // calculate all info necessary to execute a pass
    calculateInfoForPass(ball);

    /// Defenders
    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 1;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }

    /// Midfielders
    if (stpInfos["midfielder_1"].getRobot() && stpInfos["midfielder_2"].getRobot()) {
        stpInfos["midfielder_2"].setAngle((ball->getPos() - stpInfos["midfielder_2"].getRobot()->get()->getPos()).angle());
        stpInfos["midfielder_1"].setAngle((ball->getPos() - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
    }
    auto fieldWidth = field.getFieldWidth();

    auto searchGridLeft = Grid(-0.15 * fieldWidth, 0, 0.25 * fieldWidth, 1.5, 3, 3);
    auto searchGridRight = Grid(-0.25 * fieldWidth, -1.5, 0.25 * fieldWidth, 1.5, 3, 3);
    stpInfos["midfielder_1"].setPositionToMoveTo(computations::PositionComputations::determineBestOpenPosition(searchGridRight, field, world).first);
    stpInfos["midfielder_2"].setPositionToMoveTo(computations::PositionComputations::determineBestOpenPosition(searchGridLeft, field, world).first);
}

std::vector<Vector2> AttackingPass::calculateDefensivePositions(int numberOfDefenders, world::World* world, std::vector<world::view::RobotView> enemyRobots) {
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

void AttackingPass::calculateInfoForPass(const world::ball::Ball* ball) noexcept {
    /// Receiver should intercept when constraints are met
    if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionLeft.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionRight.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    }

        passingPosition = computations::PassComputations::determineBestPosForPass({receiverPositionLeft,receiverPositionRight});

    // decide kick or chip
    auto passLine = Tube(ball->getPos(), passingPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
    auto allBots = world->getWorld()->getRobotsNonOwning();
    // For all bots except passer and receivers, check if they are on the pass line, aka robot should chip
    if (std::any_of(allBots.begin(), allBots.end(), [&](const auto& bot) {
            if ((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId()) ||
                (stpInfos["receiver_left"].getRobot() && bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId()) ||
                (stpInfos["receiver_right"].getRobot() && bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
                return false;
            }
            return passLine.contains(bot->getPos());
        })) {
        stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
    } else {
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    }
    // Passer
    stpInfos["passer"].setPositionToShootAt(passingPosition);
    stpInfos["passer"].setShotType(ShotType::TARGET);
}

//bool AttackingPass::isValidPlayToStart(PlayEvaluator* playEvaluator) noexcept {
//    auto closestToBall = world->getWorld()->getRobotClosestToBall();
//    auto canKeep = std::all_of(keepPlayInvariants.begin(), keepPlayInvariants.end(), [world, field](auto& x) { return x->checkInvariant(world->getWorld().value(), &field); }) &&
//                   !passFinished();
//    if (canKeep) {
//        if (closestToBall && closestToBall->get()->getTeam() == world::us) {
//            return true;
//        } else if (world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
//            return true;
//        }
//    }
//    return false;
//}

bool AttackingPass::passFinished() noexcept {
    // TODO: improve this condition
    // Pass is done when one of the receivers is really close to the ball
    return (stpInfos["receiver_left"].getRobot() && stpInfos["receiver_left"].getRobot()->get()->getDistanceToBall() < 0.08) ||
           (stpInfos["receiver_right"].getRobot() && stpInfos["receiver_right"].getRobot()->get()->getDistanceToBall() < 0.08);
}
}  // namespace rtt::ai::stp::play
