//
// Created by jessevw on 12.03.20.
/// Places a robot between a given TARGET and a given ENEMY at a distance from the the ENEMY
/// TODO-Max de-hardcode the distance (with speed of the enemy? Standstill is block really close?)

/// PASSIVE
//

#include "stp/tactics/passive/BlockRobot.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"




#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::tactic {

BlockRobot::BlockRobot() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getEnemyRobot() || !skillStpInfo.getPositionToDefend()) return std::nullopt;

    skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));

    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value());
    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);

    return skillStpInfo;
}

double BlockRobot::calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = enemy->getPos() - targetLocation;
    return lineEnemyToTarget.angle();
}

Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation) {
    //change hard coding here
    //the block robot distance is assigned by plays
    // the values are CLOSE, HALFWAY, FAR
    //CLOSE = 1, HALFWAY = 2, FAR  = 3
    // the proportion makes the robots drive into the defense area coz they do not care t


   // very ugly hard code
    Vector2 bottomPenalty = {-4.195, -1.8};
    Vector2 topPenalty = { -4.195, 1.8};
    Vector2 aboveGoal = {-5.99, 1.8};
    Vector2 belowGoal = {-5.99, -1.8};

    double proportion;

    double m1, c1, m2, c2;
    double dx, dy;
    double intersectionX, intersectionY, intersectionXX, intersectionXY;

    dx = enemy->getPos().x - targetLocation.x;
    dy = enemy->getPos().y  - targetLocation.y;
    m1 = dy/dx;
    c1 = targetLocation.y - m1 * targetLocation.x;

    // write a proper function to determine how and where the lines intersect
    if (enemy->getPos().x > bottomPenalty.x){
        dx = topPenalty.x - bottomPenalty.x;
        dy = topPenalty.y - bottomPenalty.y;
        m2 = dx/dy;
        c2 = bottomPenalty.y - m2 * bottomPenalty.x;
        intersectionXX = (c2 - c1) / (m1 - m2);
        intersectionXY = m1 * intersectionXX + c1;
        RTT_WARNING("Bigger then x from penalty")
    } else if (enemy->getPos().y > topPenalty.y) {
        dx = topPenalty.x - aboveGoal.x;
        dy = topPenalty.y - aboveGoal.y;
        m2 = dx/dy;
        c2 = bottomPenalty.y - m2 * aboveGoal.x;
        RTT_WARNING("Bigger then Y from penalty")

    } else {
        dx = belowGoal.x - bottomPenalty.x;
        dy = belowGoal.y - bottomPenalty.y;
        m2 = dx/dy;
        c2 = bottomPenalty.y - m2 * bottomPenalty.x;
    }
    intersectionX = (c2 - c1) / (m1 - m2);
    intersectionY = m1 * intersectionX + c1;

    Vector2 intersection  = {intersectionX, intersectionY};
    Vector2 intersectionVX = {intersectionXX, intersectionXY};
    Vector2 lineEnemyToTarget;

    // put a boundary to check whether the point is in the field
    // the robots stand in the opposite direction of the actual attacker
    if (intersectionXX > aboveGoal.x && intersectionXX < -4) {
        RTT_WARNING("intersection  ", intersection);
        lineEnemyToTarget = intersectionVX - enemy->getPos();
    } else if ((intersection.y < bottomPenalty.y || intersection.y > topPenalty.y) && intersection.x > -6){
        lineEnemyToTarget = intersection - enemy->getPos();
        RTT_WARNING("target location intersection ", intersection);
    } else {
        lineEnemyToTarget = targetLocation - enemy->getPos();
        RTT_WARNING("target location ", targetLocation);

    }
    RTT_WARNING("Line enemy to target", lineEnemyToTarget)
    if (int(blockDistance) == 1) proportion = 0.5;  // adding 1 results in 0.25, 0.5, 0.75
    else if (int(blockDistance) == 2) proportion = 0.75;
    else if (int(blockDistance) == 3) proportion = 0.9;

    // // old code
//    double proportion = 0.55;
//    Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();

    auto movePosition = lineEnemyToTarget * proportion;
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
