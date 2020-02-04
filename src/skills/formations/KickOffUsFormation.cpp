#include "skills/formations/KickOffUsFormation.h"
#include <analysis/DecisionMaker.h>
#include <analysis/GameAnalyzer.h>
#include <world/FieldComputations.h>
#include "control/ControlUtils.h"

namespace rtt::ai {
std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> KickOffUsFormation::robotsInFormation = nullptr;

KickOffUsFormation::KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();
}

Vector2 KickOffUsFormation::getFormationPosition() {
    std::vector<int> robotIds;
    double fh = (*field).getFieldWidth();
    double fw = (*field).getFieldLength();

    std::vector<std::vector<Vector2>> locations = {
        {{-0.2, 0}},
        {{-0.2, 0}, {-0.2, -fh / 3}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}, {-fw / 6, 0}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}, {-fw / 6, -fh / 4}, {-fw / 7, fh / 4}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}, {-fw / 3, 0}},
        {{-0.2, 0}, {-0.2, -fh / 3}, {-0.2, fh / 3}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}, {-fw / 3, -fh / 6}, {-fw / 3, fh / 6}}};

    return getOptimalPosition(robot->id, *robotsInFormation, locations[robotsInFormation->size() - 1]);
}

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> KickOffUsFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai