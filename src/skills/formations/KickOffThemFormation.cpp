#include "skills/formations/KickOffThemFormation.h"
#include <analysis/DecisionMaker.h>
#include <analysis/GameAnalyzer.h>
#include <world/FieldComputations.h>

namespace rtt::ai {
std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormation = nullptr;

KickOffThemFormation::KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

    Vector2 KickOffThemFormation::getFormationPosition() {
        std::vector<int> robotIds;
        double fh = field->getFieldWidth();
        double fw = field->getFieldLength();

    std::vector<std::vector<Vector2>> locations = {
        {{-0.8, 0}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}, {-fw / 6, 0}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}, {-fw / 3, 0}},
        {{-0.8, 0}, {-0.4, fh / 5}, {-0.4, -fh / 5}, {-fw / 6, -fh / 4}, {-fw / 6, fh / 4}, {-fw / 7, 0}, {-fw / 3, -fh / 6}, {-fw / 3, fh / 6}}};

    return getOptimalPosition(robot->id, *robotsInFormation, locations[robotsInFormation->size() - 1]);
}

std::shared_ptr<std::vector<shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai