//
// Created by jessevw on 12.03.20.
//

#include <include/roboteam_ai/stp/new_skills/GoToPos.h>
#include <include/roboteam_ai/stp/new_skills/Rotate.h>
#include "include/roboteam_ai/stp/new_tactics/BlockRobot.h"

namespace rtt::ai::stp::tactic {

    BlockRobot::BlockRobot() {
        skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
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

    StpInfo BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
        StpInfo skillStpInfo = info;
        skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPosition().second));

        auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPosition().second);
        skillStpInfo.setPosition(std::make_pair(PositionType::MOVE_TO_POSITION, desiredRobotPosition));

        return skillStpInfo;
    }

    double BlockRobot::calculateAngle(const world_new::view::RobotView enemy, Vector2 targetLocation) {
        Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
        return lineEnemyToTarget.angle();
    }

    Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world_new::view::RobotView enemy, Vector2 targetLocation) {
        Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
        double proportion = double(blockDistance)/4;
        auto movePosition = lineEnemyToTarget*proportion;
        return movePosition + enemy->getPos();
    }

    bool BlockRobot::isEndTactic() noexcept {
        return true;
    }

    bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept {
        return false;
    }

    bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept {
        auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPosition().second);
        auto currentRobotPosition = info.getRobot().value()->getPos();
        auto cond = (desiredRobotPosition - currentRobotPosition).length() > errorMargin;
        RTT_SUCCESS(cond, 'reset')
        return cond;
    }
}
