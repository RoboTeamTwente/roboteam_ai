//
// Created by jesse on 30-04-20.
//

#include "include/roboteam_ai/stp/new_tactics/AvoidBall.h"
#include "stp/new_skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

    AvoidBall::AvoidBall() {
        skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
    }

    void AvoidBall::onInitialize() noexcept {}

    void AvoidBall::onUpdate(Status const &status) noexcept {}

    void AvoidBall::onTerminate() noexcept {
        // Call terminate on all skills
        for (auto &x : skills) {
            x->terminate();
        }
    }

    StpInfo AvoidBall::calculateInfoForSkill(StpInfo const &info) noexcept {
        StpInfo skillStpInfo = info;
        // Stretch vector between ball and robot to desired length
        auto ballToRobot = info.getRobot()->get()->getPos() - info.getBall().value()->getPos();
        auto stretchedBallToRobot = ballToRobot.stretchToLength(0.5);

        skillStpInfo.setPositionToMoveTo(info.getRobot()->get()->getPos() + stretchedBallToRobot);
        return skillStpInfo;
    }

    bool AvoidBall::isEndTactic() noexcept { return true; }

    bool AvoidBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

    bool AvoidBall::shouldTacticReset(const StpInfo &info) noexcept {
        double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
        return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
    }

    const char *AvoidBall::getName() {
        return "Avoid Ball";
    }

}  // namespace rtt::ai::stp::tactic
