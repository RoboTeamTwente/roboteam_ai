#include "skills/formations/FreeKickFormation.h"
#include <control/PositionUtils.h>

namespace rtt::ai {

std::vector<Vector2> FreeKickFormation::posses;
std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::FreeKickFormation::robotsInFormation = nullptr;

Vector2 FreeKickFormation::getFormationPosition() {
    update = true;
    posses = rtt::ai::control::PositionUtils::getFreeKickPositions(robotsInFormation->size());
    return getOptimalPosition(robot->id, *robotsInFormation, posses);
}

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> FreeKickFormation::robotsInFormationPtr() {
        return robotsInFormation;
    }

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();
};

void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}  // namespace rtt::ai
