//
// Created by jessevw on 12.03.20.
//

#include <include/roboteam_ai/stp/new_skills/GoToPos.h>
#include <include/roboteam_ai/stp/new_skills/Rotate.h>
#include "include/roboteam_ai/stp/new_tactics/BlockRobot.h"

namespace rtt::ai::stp::tactic {

    BlockRobot::BlockRobot() {
        skills = rtt::collections::state_machine<Skill, Status, StpInfo>{GoToPos(), Rotate()};
        skills.initialize();
    }

    void BlockRobot::onInitialize() noexcept {

    }

    void BlockRobot::onUpdate(Status const &status) noexcept {

    }

    void BlockRobot::onTerminate() noexcept {
        // Call terminate on all skills
        for (auto &x : skills) {
            x->terminate();
        }
    }

    // TODO: add blockDistance to STPInfo and calculate it
    // TODO: Calculate the angle to be facing the second position
    StpInfo BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
        StpInfo skillStpInfo = info;
        skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getTargetPos().second));
        auto moveTarget = calculateMoveTarget(info.getBlockDistance(), info.getEnemyRobot().value(), info.getTargetPos().second);
        skillStpInfo.setTargetPos(std::make_pair(TargetType::MOVETARGET, moveTarget));
        return skillStpInfo;
    }

    double BlockRobot::calculateAngle(const world_new::view::RobotView enemy, Vector2 targetLocation) {
        Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
        return lineEnemyToTarget.angle();

    }

    Vector2 BlockRobot::calculateMoveTarget(BlockDistance blockDistance, const world_new::view::RobotView enemy, Vector2 targetLocation) {
        Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
        double proportion = double(blockDistance)/4;
        auto movePosition = lineEnemyToTarget*proportion;
        return movePosition + enemy->getPos();
    }
}