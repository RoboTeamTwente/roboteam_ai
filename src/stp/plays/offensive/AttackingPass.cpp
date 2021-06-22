//
// Created by jessevw on 17.03.20.
/// TODO-Max change to fowardPass
//

#include <roboteam_utils/Tube.h>

#include "stp/plays/offensive/AttackingPass.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PassComputations.h"

#include <stp/roles/passive/Defender.h>
#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"

namespace rtt::ai::stp::play {
    AttackingPass::AttackingPass() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        startPlayEvaluation.emplace_back(GlobalEvaluation::BallCloseToUs);
        startPlayEvaluation.emplace_back(GlobalEvaluation::NoGoalVisionFromBall);
        startPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        keepPlayEvaluation.emplace_back(GlobalEvaluation::NoGoalVisionFromBall);
        keepPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);


        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::Passer>(role::Passer("passer")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_left")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_right")),
                std::make_unique<role::Formation>(role::Formation("midfielder_0")),
                std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                std::make_unique<role::Formation>(role::Formation("waller_0")),
                std::make_unique<role::Formation>(role::Formation("waller_1")),
                std::make_unique<role::Defender>(role::Defender("defender_0")),
                std::make_unique<role::Defender>(role::Defender("defender_1"))};

        // initialize stpInfos
        stpInfos = std::unordered_map<std::string, StpInfo>{};
        for (auto &role : roles) {
            role->reset();
            auto roleName{role->getName()};
            stpInfos.emplace(roleName, StpInfo{});
        }
    }

    uint8_t AttackingPass::score(PlayEvaluator &playEvaluator) noexcept {
        //TODO: Uncomment this (and actually implement the scoring) when you want to score this play based on
        // how good the attacking pass will be
        //calculateInfoForScoredRoles(playEvaluator.getWorld());

        scoring = {{playEvaluator.getGlobalEvaluation(GlobalEvaluation::BallCloseToUs), 1}};
        //std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::GoalVisionFromBall), 1)};
        //std::make_pair(std::max({stpInfos["receiver_left"].getRoleScore().value(),stpInfos["receiver_right"].getRoleScore().value()}),1)};
        return (lastScore = playEvaluator.calculateScore(scoring)).value();
    }

    Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
        Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {passerFlag}}});
        flagMap.insert({"receiver_left", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
        flagMap.insert({"receiver_right", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
        flagMap.insert({"midfielder_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"waller_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    void AttackingPass::calculateInfoForScoredRoles(world::World *world) noexcept {
        rtt::world::Field field = world->getField().value();

        // Receiver
        stpInfos["receiver_left"].setPositionToMoveTo(
                PositionComputations::getPosition(stpInfos["receiver_left"].getPositionToMoveTo(),
                                                  gen::gridRightBot, gen::GoalShootPosition, field, world));
        stpInfos["receiver_right"].setPositionToMoveTo(
                PositionComputations::getPosition(stpInfos["receiver_right"].getPositionToMoveTo(),
                                                  gen::gridRightTop, gen::GoalShootPosition, field, world));
    }

    void AttackingPass::calculateInfoForRoles() noexcept {
        auto ball = world->getWorld()->getBall()->get();
//        calculateInfoForScoredRoles(world);
        /// Keeper
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
        stpInfos["keeper"].setPositionToShootAt(
                world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
        stpInfos["keeper"].setKickOrChip(KickOrChip::CHIP);

        /// Passer and receivers
        // calculate all info necessary to execute a pass
        calculateInfoForPass(ball);

        /// Defenders
        auto enemyRobots = world->getWorld()->getThem();

        stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["defender_0"].setEnemyRobot(enemyRobots[0]);
        stpInfos["defender_0"].setBlockDistance(BlockDistance::CLOSE);
        //(stpInfos.find("defender_0")->second.getRobot()->get()->getPos()-enemyRobots[0].get()->getPos()).length()/2

        stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_1"].setEnemyRobot(enemyRobots[1]);
        stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

        /// Wallers that will block the line from the ball to the goal
        stpInfos["waller_0"].setPositionToMoveTo(pos::getWallPosition(0, 2, field, world));
        stpInfos["waller_1"].setPositionToMoveTo(pos::getWallPosition(1, 2, field, world));

        /// Slightly aggressive midfielders
        stpInfos["midfielder_0"].setPositionToMoveTo(
                PositionComputations::getPosition(stpInfos["midfielder_0"].getPositionToMoveTo(),
                                                  gen::gridMidFieldMid, gen::OffensivePosition, field, world));
        stpInfos["midfielder_1"].setPositionToMoveTo(
                PositionComputations::getPosition(stpInfos["midfielder_1"].getPositionToMoveTo(),
                                                  gen::gridMidFieldBot, gen::OffensivePosition, field, world));
        stpInfos["midfielder_2"].setPositionToMoveTo(
                PositionComputations::getPosition(stpInfos["midfielder_2"].getPositionToMoveTo(),
                                                  gen::gridMidFieldTop, gen::OffensivePosition, field, world));
    }

    std::vector<Vector2> AttackingPass::calculateDefensivePositions(int numberOfDefenders,
                                                                    std::vector<world::view::RobotView> enemyRobots) {
        std::vector<Vector2> positions = {};
        positions.reserve(numberOfDefenders);

        for (int i = 0; i < numberOfDefenders; i++) {
            positions.push_back(enemyRobots[i].get()->getPos());
        }
        return positions;
    }

    const char *AttackingPass::getName() { return "AttackingPass"; }

// TODO: This function should be split, the passing for this play and the receiving in another play
    void AttackingPass::calculateInfoForPass(const world::ball::Ball *ball) noexcept {
        /// Recalculate pass positions if we did not shoot yet
        /// For the receive locations, divide the field up into grids where the passers should stand,
        /// and find the best locations in those grids
        auto receiverPositionRight = PositionComputations::getPosition(
                stpInfos["receiver_right"].getPositionToMoveTo(), gen::gridRightBot, gen::GoalShootPosition, field,
                world);
        auto receiverPositionLeft = PositionComputations::getPosition(
                stpInfos["receiver_left"].getPositionToMoveTo(), gen::gridRightTop, gen::GoalShootPosition, field,
                world);

        stpInfos["passer"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos());

        std::vector<gen::ScoredPosition> positions = {receiverPositionLeft, receiverPositionRight};
        Vector2 passLocation = computations::PassComputations::determineBestPosForPass(positions);

        /// If no good pass found, pass to closest robot
        if (passLocation == Vector2{0, 0}) {
            passLocation = world->getWorld()->getRobotClosestToPoint(stpInfos["passer"].getPositionToMoveTo().value(),
                                                                     world::us)->get()->getPos();
        }

        /// Receiver should intercept when constraints are met
        if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
            receiverPositionLeft.position = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(
                    passLocation);
        } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
            receiverPositionRight.position = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(
                    passLocation);
        }

        // Receiver
        stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.position);
        stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.position);

        // decide kick or chip
        auto passLine = Tube(ball->getPos(), passLocation, control_constants::ROBOT_CLOSE_TO_POINT / 2);
        auto allBots = world->getWorld()->getRobotsNonOwning();
        // For all bots except passer and receivers, check if they are on the pass line, aka robot should chip
        if (std::any_of(allBots.begin(), allBots.end(), [&](const auto &bot) {
            if ((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId()) ||
                (stpInfos["receiver_left"].getRobot() &&
                 bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId()) ||
                (stpInfos["receiver_right"].getRobot() &&
                 bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
                return false;
            }
            return passLine.contains(bot->getPos());
        })) {
            stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
        } else {
            stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
        }
        // Passer
        stpInfos["passer"].setPositionToShootAt(passLocation);
        stpInfos["passer"].setShotType(ShotType::PASS);
    }

/// To be reimplemented, removed as the implementation of the default isValidPlayToStart is changing
    bool AttackingPass::isValidPlayToStart(PlayEvaluator *playEvaluator) noexcept {
        auto closestToBall = world->getWorld()->getRobotClosestToBall();

        auto canKeep = std::all_of(keepPlayEvaluation.begin(), keepPlayEvaluation.end(),
                                   [playEvaluator](auto &x) { return playEvaluator->checkEvaluation(x); }) &&
                       !passFinished();

        if (canKeep) {
            if (closestToBall && closestToBall->get()->getTeam() == world::us) {
                return true;
            } else if (world->getWorld()->getBall().value()->getVelocity().length() >
                       control_constants::BALL_STILL_VEL) {
                return true;
            }
        }
        return false;
    }

    bool AttackingPass::passFinished() noexcept {
        // TODO: improve this condition
        // Pass is done when one of the receivers is really close to the ball
        return (stpInfos["receiver_left"].getRobot() &&
                stpInfos["receiver_left"].getRobot()->get()->getDistanceToBall() < 0.08) ||
               (stpInfos["receiver_right"].getRobot() &&
                stpInfos["receiver_right"].getRobot()->get()->getDistanceToBall() < 0.08);
    }

    void AttackingPass::storePlayInfo(gen::PlayInfos &info) noexcept {
        gen::StoreInfo passer;
        passer.robotID = stpInfos["passer"].getRobot()->get()->getId();
        passer.passToRobot = stpInfos["passer"].getPositionToShootAt();
        info.insert({gen::KeyInfo::isPasser, passer});
    }
}  // namespace rtt::ai::stp::play
