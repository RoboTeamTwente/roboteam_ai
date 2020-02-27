#include <control/PositionUtils.h>
#include <skills/formations/FreeKickFormation.h>

namespace rtt::ai {

std::vector<Vector2> FreeKickFormation::posses;
std::vector<world_new::view::RobotView> rtt::ai::FreeKickFormation::robotsInFormation{};

Vector2 FreeKickFormation::getFormationPosition() {
    update = true;
    posses = rtt::ai::control::PositionUtils::getFreeKickPositions(*field, robotsInFormation.size());
    return getOptimalPosition(robot->get()->getId(), robotsInFormation, posses);
}

std::vector<world_new::view::RobotView> FreeKickFormation::robotsInFormationPtr() { return robotsInFormation; }

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
};

void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}  // namespace rtt::ai
