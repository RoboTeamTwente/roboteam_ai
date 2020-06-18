//
// Created by timovdk on 5/20/20.
//

#include "stp/new_plays/GenericPass.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallOnOurSideInvariant.h"
#include "stp/invariants/FreedomOfRobotsInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/Halt.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"

namespace rtt::ai::stp::play {

GenericPass::GenericPass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::FreedomOfRobotsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_0")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_1")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_2")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_7"))};
}

uint8_t GenericPass::score(world_new::World* world) noexcept { return 100; }

void GenericPass::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));

    const Vector2 passingPosition = calculatePassLocation();

    auto ball = world->getWorld()->getBall().value();

    auto receivePosition = passingPosition;
    if ((ball->getPos() - passingPosition).length() <= 2.0 && ball->getVelocity().length() > 0.1) {
       receivePosition = ball->getPos();
    }

    // Receiver
    stpInfos["receiver"].setPositionToMoveTo(receivePosition);

    // Passer
    stpInfos["passer"].setPositionToShootAt(passingPosition);
    stpInfos["passer"].setKickChipType(TARGET);
}

bool GenericPass::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GenericPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"receiver", {receiverFlag}});
    flagMap.insert({"halt_0", {not_important}});
    flagMap.insert({"halt_1", {not_important}});
    flagMap.insert({"halt_2", {not_important}});
    flagMap.insert({"halt_3", {not_important}});
    flagMap.insert({"halt_4", {not_important}});
    flagMap.insert({"halt_5", {not_important}});
    flagMap.insert({"halt_6", {not_important}});
    flagMap.insert({"halt_7", {not_important}});
    return flagMap;
}

const char* GenericPass::getName() { return "Generic Pass"; }

const Vector2 GenericPass::calculatePassLocation() const noexcept {
    double offSetX = 0;  // start looking for suitable positions to move to at 30% of the field width
    double offSetY = -3;
    double regionWidth = 3;
    double regionHeight = 6;
    auto numStepsX = 20;
    auto numStepsY = 20;

    double bestScore{};
    Vector2 bestPosition{};

    auto w = world->getWorld().value();

    // Make a grid with all potentially good points
    Grid grid = Grid(offSetX, offSetY, regionWidth, regionHeight, numStepsX, numStepsY);
    for (const auto& nestedPoints : grid.getPoints()) {
        for (const auto& trial : nestedPoints) {
            // Make sure we only check valid points
            if (!FieldComputations::pointIsInDefenseArea(field, trial, false)) {
                auto fieldWidth = field.getFieldWidth();
                auto fieldLength = field.getFieldLength();
                auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);

                // Make sure the ball can reach the target
                auto canReachTarget{1.0};
                auto passLine = Tube(w->getBall()->get()->getPos(), trial, control_constants::ROBOT_CLOSE_TO_POINT);
                auto enemyBots = w.getThem();
                if (std::any_of(enemyBots.begin(), enemyBots.end(), [&](const auto& bot) { return passLine.contains(bot->getPos()); })) {
                    canReachTarget = 0.0;
                }

                // Search closest bot to this point and get that distance
                auto theirClosestBot = w.getRobotClosestToPoint(trial, world_new::Team::them);
                auto theirClosestBotDistance{1.0};
                if(theirClosestBot) {
                    theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial) / fieldDiagonalLength;
                }

                // Calculate total score for this point
                auto pointScore = 0.5 * theirClosestBotDistance * canReachTarget;

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

bool GenericPass::isValidPlayToKeep(world_new::World *world) noexcept {
    world::Field field = world->getField().value();
    return std::all_of(keepPlayInvariants.begin(), keepPlayInvariants.end(), [world, field](auto &x){return x->checkInvariant(world->getWorld().value(), &field);}) && !passFinished() && !passFailed();
}

bool GenericPass::passFinished() noexcept {
    //TODO: fix this condition
    if(stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot()->get()->getDistanceToBall() < 0.5) {
        return true;
    }
    return false;
}

bool GenericPass::passFailed() noexcept {
    //TODO: fix this condition
    if(stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot()->get()->getAngleDiffToBall() > M_PI_4) {
        return true;
    }
    return false;
}
}  // namespace rtt::ai::stp::play
