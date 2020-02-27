//
// Created by baris on 24-4-19.
//

#include <control/PositionUtils.h>
#include <skills/formations/DefendFreeKick.h>

namespace rtt::ai {

std::vector<Vector2> DefendFreeKick::posses;
std::vector<world_new::view::RobotView> rtt::ai::DefendFreeKick::robotsInFormation{};

Vector2 DefendFreeKick::getFormationPosition() {
    robot->getControllers().getNumTreePosController()->setAvoidBallDistance(0.55);

    update = true;
    posses = rtt::ai::control::PositionUtils::getDefendFreeKick(ball.value(), *field, robotsInFormation.size());

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, posses);
}

std::vector<world_new::view::RobotView> DefendFreeKick::robotsInFormationPtr() { return robotsInFormation; }

DefendFreeKick::DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard) : Formation(std::move(name), std::move(blackboard)) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
}

void DefendFreeKick::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}  // namespace rtt::ai
