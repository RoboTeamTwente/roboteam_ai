//
// Created by timovdk on 3/16/20.
//

#include <stp/new_skills/GoToPos.h>
#include <stp/new_skills/Rotate.h>
#include <stp/new_tactics/DriveWithBall.h>

namespace rtt::ai::stp::tactic {

DriveWithBall::DriveWithBall() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::GoToPos()};
    skills.initialize();
}

void DriveWithBall::onInitialize() noexcept {}

void DriveWithBall::onUpdate(Status const& status) noexcept {}

void DriveWithBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto& x : skills) {
        x->terminate();
    }
}

StpInfo DriveWithBall::calculateInfoForSkill(StpInfo const& info) noexcept {
    StpInfo skillStpInfo = info;

    double angleToBall = (info.getTargetPos().second - info.getBall()->get()->getPos()).angle();
    skillStpInfo.setAngle(angleToBall);


    // When driving with ball, we need to activate the dribbler
    // For now, this means full power, but this might change later
    // TODO better way to determine dribblerspeed
    skillStpInfo.setDribblerSpeed(31);

    return skillStpInfo;
}

bool DriveWithBall::isTacticFailing(const StpInfo& info) noexcept { return !info.getRobot()->hasBall() || info.getTargetPos().first != MOVETARGET; }

bool DriveWithBall::shouldTacticReset(const StpInfo& info) noexcept {
    return fabs(info.getRobot()->get()->getAngle() + (info.getBall()->get()->getPos() - info.getRobot()->get()->getPos()).angle()) <= Constants::GOTOPOS_ANGLE_ERROR_MARGIN();
}
}  // namespace rtt::ai::stp::tactic