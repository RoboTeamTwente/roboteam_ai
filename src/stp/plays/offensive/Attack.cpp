//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "stp/plays/offensive/Attack.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(("keeper")),
                                                                                       std::make_unique<role::Attacker>(("striker")),
                                                                                       std::make_unique<role::Formation>(("waller_1")),
                                                                                       std::make_unique<role::Formation>(("waller_2")),
                                                                                       std::make_unique<role::Formation>(("midfielder_mid")),
                                                                                       std::make_unique<role::Formation>(("attacker_1")),
                                                                                       std::make_unique<role::Formation>(("attacker_2")),
                                                                                       std::make_unique<role::Formation>(("midfielder_left")),
                                                                                       std::make_unique<role::Formation>(("midfielder_right")),
                                                                                       std::make_unique<role::Formation>(("attacking_midfielder")),
                                                                                       std::make_unique<role::BallDefender>(("defender_right"))};
}

uint8_t Attack::score(const rtt::world::Field& field) noexcept {
    // Score the position of the ball based on the odds of scoring
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);


    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});                                                         //
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});  //
    flagMap.insert({"attacker_2", {DealerFlagPriority::LOW_PRIORITY, {kickerFirstPriority, kickerSecondPriority}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacking_midfielder", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::MEDIUM_PRIORITY, {kickerFirstPriority, kickerSecondPriority}}});             //


    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    calculateInfoForAttackers();
    calculateInfoForMidfielders();
    calculateInfoForDefenders();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // Striker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    goalTarget.y = std::clamp(goalTarget.y, field.getTheirBottomGoalSide().y + 0.4, field.getTheirTopGoalSide().y - 0.4);
    stpInfos["striker"].setPositionToShootAt(goalTarget);
    stpInfos["striker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["striker"].setShotType(ShotType::MAX);
}

void Attack::calculateInfoForDefenders() noexcept {
    constexpr auto wallerNames = std::array{"waller_1", "waller_2"};
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
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);
}

void Attack::calculateInfoForMidfielders() noexcept {
    stpInfos["midfielder_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_mid"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::BlockingPosition, field, world));
    stpInfos["midfielder_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));

    // If the ball (and therefore striker) are in the front of the field, let the attacking midfielder go to the midfield
    // If the striker is not in the front field already, let the attacking midfielder go to the free section in the front field
    if (world->getWorld()->getBall()->get()->position.x > field.getFrontMidGrid().getOffSetX()) {
        stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    } else {
        if (world->getWorld()->getBall().value()->position.y > field.getFrontLeftGrid().getOffSetY()) {  // Ball is in left of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        } else if (world->getWorld()->getBall().value()->position.y < field.getFrontMidGrid().getOffSetY()) {  // Ball is in right of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
        } else {  // Ball is in middle of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
        }
    }
}

void Attack::calculateInfoForAttackers() noexcept {
    // Set the attackers to go to the part of the field where the ball is NOT (in y-direction), since that is where the striker will be
    if (world->getWorld()->getBall().value()->position.y > field.getFrontLeftGrid().getOffSetY()) {  // Ball is in left of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    } else if (world->getWorld()->getBall().value()->position.y < field.getFrontMidGrid().getOffSetY()) {  // Ball is in right of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
    } else {  // Ball is in middle of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    }
}

bool Attack::shouldEndPlay() noexcept {
    if  (std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "striker" && role->finished(); }))
        return true;

    // TODO: probably should do this via an invariant (BALL_IS_NOT_MOVING_FAST or something similar)
    if (world->getWorld()->getBall()->get()->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT)
        return true;

    return false;
}

const char* Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
