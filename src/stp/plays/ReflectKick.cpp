//
// Created by jordi on 19-05-20.
//

#include "stp/plays/ReflectKick.h"

#include "stp/evaluations/global/BallCloseToUsGlobalEvaluation.h"
#include "stp/evaluations/global/BallClosestToUsGlobalEvaluation.h"
#include "stp/evaluations/global/WeHaveBallGlobalEvaluation.h"
#include "stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h"
#include "stp/roles/active/BallReflector.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Passer.h"

namespace rtt::ai::stp::play {

    ReflectKick::ReflectKick() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
        startPlayEvaluation.emplace_back(eval::WeHaveBall);
        startPlayEvaluation.emplace_back(eval::BallClosestToUs);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
        keepPlayEvaluation.emplace_back(eval::BallCloseToUs);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
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

    uint8_t ReflectKick::score(PlayEvaluator &playEvaluator) noexcept {

        auto closestBot = playEvaluator.getWorld()->getWorld()->getRobotClosestToBall(world::us);
        auto sum = 0;
        auto _field = playEvaluator.getWorld()->getField();
        if(_field.has_value()){
            std::vector<world::view::RobotView> potentialBots = {};
            for (auto robot : playEvaluator.getWorld()->getWorld()->getUs()) {
                if (robot->getPos().x < closestBot->get()->getPos().x &&
                    robot->getPos().dist(_field->getTheirGoalCenter()) < _field->getFieldLength() / 4) {
                    potentialBots.push_back(robot);
                    sum += 30;
                }
            }
        }
        return sum;
    }

    Dealer::FlagMap ReflectKick::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL,
                                                DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"reflector", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag}}});
        flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
        flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"offender_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"midfielder_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});

        return flagMap;
    }

    void ReflectKick::calculateInfoForRoles() noexcept {
        // TODO: Change this
        // TODO: Leave play when reflect kick fails
        auto passPosition = Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4);

        auto ball = world->getWorld()->getBall().value();
        std::optional<Vector2> intersection;

        if ((ball->getPos() - passPosition).length() <= 2.0 && ball->getVelocity().length() > 0.1) {
            LineSegment ballDirection = LineSegment(ball->getPos(), ball->getPos() + ball->getVelocity());
            LineSegment betweenPassAndGoal = LineSegment(passPosition, field.getTheirGoalCenter());
            intersection = ballDirection.intersects(betweenPassAndGoal);
        }

        // First estimate of where the reflect kick location should be
        auto intermediateReflectPosition = (intersection.has_value() &&
                                            FieldComputations::pointIsInField(field, intersection.value()))
                                           ? intersection.value() : passPosition;
        auto reflectPositionToGoal = intermediateReflectPosition - field.getTheirGoalCenter();
        // The final reflect kick position is stretched to account for the robot diameter
        auto reflectPosition = field.getTheirGoalCenter() + (reflectPositionToGoal).stretchToLength(
                (reflectPositionToGoal).length() + control_constants::CENTER_TO_FRONT);

        // Reflector
        stpInfos["reflector"].setPositionToMoveTo(reflectPosition);
        stpInfos["reflector"].setPositionToShootAt(field.getTheirGoalCenter());
        stpInfos["reflector"].setShotType(ShotType::PASS);

        for (auto &role : roles) {
            if (role->getName() == "reflector") {
                if (!role->finished() && strcmp(role->getCurrentTactic()->getName(), "Position And Aim") == 0 &&
                    stpInfos["reflector"].getRobot().has_value() &&
                    stpInfos["reflector"].getRobot().value()->getDistanceToBall() <= control_constants::BALL_IS_CLOSE) {
                    role->forceNextTactic();
                }
            }
        }

        // Passer
        stpInfos["passer"].setPositionToShootAt(field.getTheirGoalCenter());
        stpInfos["passer"].setShotType(ShotType::PASS);
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
        stpInfos["defender_1"].setEnemyRobot(
                world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
        stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

        stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
        stpInfos["defender_2"].setEnemyRobot(
                world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
        stpInfos["defender_2"].setBlockDistance(BlockDistance::CLOSE);

        stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_3"].setEnemyRobot(
                world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
        stpInfos["defender_3"].setBlockDistance(BlockDistance::CLOSE);

        // Keeper
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
        stpInfos["keeper"].setPositionToShootAt(Vector2());
    }

    const char *ReflectKick::getName() { return "Reflect Kick"; }

}  // namespace rtt::ai::stp::play
