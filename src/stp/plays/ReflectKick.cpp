//
// Created by jordi on 19-05-20.
//

#include "stp/plays/ReflectKick.h"

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallClosestToUsInvariant.h"
#include "stp/invariants/WeHaveBallInvariant.h"
#include "stp/invariants/game_states/NormalOrFreeKickUsGameStateInvariant.h"
#include "stp/roles/BallReflector.h"
#include "stp/roles/Defender.h"
#include "stp/roles/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/Passer.h"

namespace rtt::ai::stp::play {

ReflectKick::ReflectKick() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallReflector>(role::BallReflector("reflector")),
                                                                                 std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_3")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_3"))};
}

uint8_t ReflectKick::score(world_new::World *world) noexcept {
    auto closestBot = world->getWorld()->getRobotClosestToBall(world_new::us);
    auto sum = 0;
    std::vector<world_new::view::RobotView> potentialBots = {};
    for (auto robot : world->getWorld()->getUs()) {
        if (robot->getPos().x < closestBot->get()->getPos().x && robot->getPos().dist(field.getTheirGoalCenter()) < field.getFieldLength() / 4) {
            potentialBots.push_back(robot);
            sum += 30;
        }
    }
    return sum;
}

Dealer::FlagMap ReflectKick::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"reflector", {closeToTheirGoalFlag}});
    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"offender_1", {notImportant}});
    flagMap.insert({"offender_2", {notImportant}});
    flagMap.insert({"midfielder_1", {notImportant}});
    flagMap.insert({"midfielder_2", {notImportant}});
    flagMap.insert({"midfielder_3", {notImportant}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"defender_3", {closeToOurGoalFlag}});

    return flagMap;
}

void ReflectKick::calculateInfoForRoles() noexcept {
    // TODO: Change this
    // TODO: Leave play when reflect kick fails
    auto passPosition = Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4);

    auto ball = world->getWorld()->getBall().value();
    std::optional<Vector2> intersection;

    if ((ball->getPos() - passPosition).length() <= 2.0 && ball->getVelocity().length() > 0.1) {
        intersection = Line(ball->getPos(), ball->getPos() + ball->getVelocity()).intersects(Line(passPosition, field.getTheirGoalCenter()));
    }

    // First estimate of where the reflect kick location should be
    auto intermediateReflectPosition = (intersection.has_value() && FieldComputations::pointIsInField(field, intersection.value())) ? intersection.value() : passPosition;
    auto reflectPositionToGoal = intermediateReflectPosition - field.getTheirGoalCenter();
    // The final reflect kick position is stretched to account for the robot diameter
    auto reflectPosition = field.getTheirGoalCenter() + (reflectPositionToGoal).stretchToLength((reflectPositionToGoal).length() + control_constants::CENTER_TO_FRONT);

    // Reflector
    stpInfos["reflector"].setPositionToMoveTo(reflectPosition);
    stpInfos["reflector"].setPositionToShootAt(field.getTheirGoalCenter());
    stpInfos["reflector"].setShotType(ShotType::MAX);

    for (auto &role : roles) {
        if (role->getName() == "reflector") {
            if (!role->finished() && strcmp(role->getCurrentTactic()->getName(), "Position And Aim") == 0 && stpInfos["reflector"].getRobot().has_value() &&
                stpInfos["reflector"].getRobot().value()->getDistanceToBall() <= control_constants::BALL_IS_CLOSE) {
                role->forceNextTactic();
            }
        }
    }

    // Passer
    stpInfos["passer"].setPositionToShootAt(passPosition);
    stpInfos["passer"].setShotType(ShotType::TARGET);
    stpInfos["passer"].setKickOrChip(KickOrChip::KICK);

    // Offenders
    stpInfos["offender_1"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, field.getFieldWidth() / 4));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4));

    // Midfielders
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, field.getFieldWidth() / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -field.getFieldWidth() / 4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(field.getFieldLength() / 8, 0.0));

    // Defenders
    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world_new::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_3"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world_new::them));
    stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);

    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

bool ReflectKick::shouldRoleSkipEndTactic() { return false; }

const char *ReflectKick::getName() { return "Reflect Kick"; }

}  // namespace rtt::ai::stp::play