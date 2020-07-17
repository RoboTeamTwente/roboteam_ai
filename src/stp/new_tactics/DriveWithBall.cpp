//
// Created by timovdk on 3/16/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_skills/GoToPos.h>
#include <stp/new_skills/Rotate.h>
#include <stp/new_tactics/DriveWithBall.h>

namespace rtt::ai::stp::tactic {

DriveWithBall::DriveWithBall() {
  // Create state machine of skills and initialize first skill
  // TODO: The rotate skill might be unnecessary in this tactic if our dribbler works well whilst driving backwards
  skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::GoToPos()};
}

std::optional<StpInfo> DriveWithBall::calculateInfoForSkill(StpInfo const& info) noexcept {
  StpInfo skillStpInfo = info;

  if (!skillStpInfo.getPositionToMoveTo() || !skillStpInfo.getBall()) return std::nullopt;

  double angleToBall = (info.getPositionToMoveTo().value() - info.getBall()->get()->getPos()).angle();
  skillStpInfo.setAngle(angleToBall);

  // When driving with ball, we need to activate the dribbler
  // For now, this means full power, but this might change later
  // TODO: TUNE better way to determine dribblerspeed
  skillStpInfo.setDribblerSpeed(100);

  return skillStpInfo;
}

bool DriveWithBall::isTacticFailing(const StpInfo& info) noexcept {
  // Fail if we don't have the ball or there is no movement position
  return !info.getRobot()->hasBall() || !info.getPositionToMoveTo();
}

bool DriveWithBall::shouldTacticReset(const StpInfo& info) noexcept {
  // Should reset if the angle the robot is at is no longer correct
  auto ballToRobotAngle = (info.getBall()->get()->getPos() - info.getRobot()->get()->getPos()).angle();
  double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
  return info.getRobot().value()->getAngle().shortestAngleDiff(ballToRobotAngle) > errorMargin;
}

bool DriveWithBall::isEndTactic() noexcept {
  // This is not an end tactic
  return false;
}

const char* DriveWithBall::getName() { return "Drive With Ball"; }

}  // namespace rtt::ai::stp::tactic
