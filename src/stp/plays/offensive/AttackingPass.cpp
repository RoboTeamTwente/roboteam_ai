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

namespace rtt::ai::stp::play {
AttackingPass::AttackingPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
//    startPlayEvaluation.emplace_back(GlobalEvaluation::BallCloseToUs);
//    startPlayEvaluation.emplace_back(GlobalEvaluation::NoGoalVisionFromBall);
//    startPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallOnOurSide);


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

uint8_t AttackingPass::score(PlayEvaluator& playEvaluator) noexcept {
    calculateInfoForScoredRoles(playEvaluator.getWorld());
    scoring = {{playEvaluator.getGlobalEvaluation(GlobalEvaluation::BallCloseToUs), 1}};
               //std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::GoalVisionFromBall), 1)};
               //std::make_pair(std::max({stpInfos["receiver_left"].getRoleScore().value(),stpInfos["receiver_right"].getRoleScore().value()}),1)};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();
    }

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER,{}}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED,{passerFlag},5}});
    flagMap.insert({"receiver_left", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}, 6}});
    flagMap.insert({"receiver_right", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"test_role_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"test_role_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForScoredRoles(world::World* world) noexcept {
    rtt::world::Field field = world->getField().value();

    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["receiver_left"].getPositionToMoveTo(), gen::gridRightBot, gen::GoalShootPosition, field, world));
    stpInfos["receiver_right"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["receiver_right"].getPositionToMoveTo(),gen::gridRightTop, gen::GoalShootPosition, field, world));
    }

void AttackingPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();
    calculateInfoForScoredRoles(world);
    /// Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    /// Passer and receivers
    // calculate all info necessary to execute a pass
    //calculateInfoForPass(ball);

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
    stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["midfielder_1"].getPositionToMoveTo(),gen::gridMidFieldBot, gen::SafePosition, field, world));
    stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["midfielder_2"].getPositionToMoveTo(),gen::gridMidFieldTop, gen::SafePosition, field, world));
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

/// Should become new play that receives ball
//void AttackingPass::calculateInfoForPass(const world::ball::Ball* ball) noexcept {
//    /// Recalculate pass positions if we did not shoot yet
//        /// For the receive locations, divide the field up into grids where the passers should stand,
//        /// and find the best locations in those grids
//        receiverPositionRight = computations::PositionComputations::determineBestGoalShotLocation(gridRight, field, world);
//        receiverPositionLeft = computations::PositionComputations::determineBestGoalShotLocation(gridLeft, field, world);
//        std::vector<computations::PositionComputations::PositionEvaluation> positions = {receiverPositionLeft,receiverPositionRight};
//
//        Vector2 passLocation = computations::PassComputations::determineBestPosForPass(positions);
//    /// Receiver should intercept when constraints are met
//    if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
//        receiverPositionLeft.position = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
//    } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
//        receiverPositionRight.position = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
//    }
//
//    // Receiver
//    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.position);
//    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.position);
//
//    // decide kick or chip
//    auto passLine = Tube(ball->getPos(), passingPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
//    auto allBots = world->getWorld()->getRobotsNonOwning();
//    // For all bots except passer and receivers, check if they are on the pass line, aka robot should chip
//    if (std::any_of(allBots.begin(), allBots.end(), [&](const auto& bot) {
//            if ((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId()) ||
//                (stpInfos["receiver_left"].getRobot() && bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId()) ||
//                (stpInfos["receiver_right"].getRobot() && bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
//                return false;
//            }
//            return passLine.contains(bot->getPos());
//        })) {
//        stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
//    } else {
//        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
//    }
//    // Passer
//    stpInfos["passer"].setPositionToShootAt(passingPosition);
//    stpInfos["passer"].setShotType(ShotType::TARGET);
//}

/// To be reimplemented, removed as the implementation of the default isValidPlayToStart is changing
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

   void AttackingPass::storePlayInfo(PlayInfos& info) noexcept {
        StoreInfo passer;
        passer.robotID = stpInfos["passer"].getRobot()->get()->getId();
        passer.passToRobot = stpInfos["passer"].getPositionToShootAt();
        info.insert({KeyInfo::isPasser,passer});
    }
}  // namespace rtt::ai::stp::play
