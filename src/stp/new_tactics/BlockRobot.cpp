//
// Created by jessevw on 12.03.20.
//

#include "include/roboteam_ai/stp/new_tactics/BlockRobot.h"
#include <include/roboteam_ai/stp/new_skills/GoToPos.h>
#include <include/roboteam_ai/stp/new_skills/Rotate.h>

namespace rtt::ai::stp::tactic {

BlockRobot::BlockRobot() {
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void BlockRobot::onInitialize() noexcept {}

void BlockRobot::onUpdate(Status const &status) noexcept {}

void BlockRobot::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

std::optional<StpInfo> BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if(!skillStpInfo.getEnemyRobot() || !skillStpInfo.getPositionToDefend()) return std::nullopt;

    skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));

    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value());
    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);

    return skillStpInfo;
}

double BlockRobot::calculateAngle(const world_new::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = enemy->getPos() - targetLocation;
    return lineEnemyToTarget.angle();
}

Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world_new::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
    double proportion = double(blockDistance) / (blockEnumSize + 1);  // adding 1 results in 0.25, 0.5, 0.75
    auto movePosition = lineEnemyToTarget * proportion;
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset if the robot is too far from its desired location
    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value());
    auto currentRobotPosition = info.getRobot().value()->getPos();
    auto cond = (desiredRobotPosition - currentRobotPosition).length() > errorMargin;
    return cond;
}

const char *BlockRobot::getName() {
    return "Block Robot";
}

}  // namespace rtt::ai::stp::tactic
