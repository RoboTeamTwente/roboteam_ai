//
// Created by jesse on 30-04-20.
//

#include "stp/new_tactics/AvoidBall.h"
#include "stp/new_skills/GoToPos.h"
#include "world/FieldComputations.h"

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
        Vector2 stretchedBallToRobot;
        if (info.getAvoidBallDistance()) {
            stretchedBallToRobot = ballToRobot.stretchToLength(info.getAvoidBallDistance().value());
        }
        else {
            stretchedBallToRobot = ballToRobot.stretchToLength(0.5);
        }

        auto positionFromBall = info.getRobot()->get()->getPos() + stretchedBallToRobot;

        auto field = info.getField().value();
        if (!rtt::ai::FieldComputations::pointIsInField(field, positionFromBall, control_constants::ROBOT_RADIUS_MAX)){

        }

        skillStpInfo.setPositionToMoveTo(positionFromBall);



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
