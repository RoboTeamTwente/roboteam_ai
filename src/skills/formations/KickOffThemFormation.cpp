#include <skills/formations/KickOffThemFormation.h>

namespace rtt::ai {
std::vector<world_new::view::RobotView> KickOffThemFormation::robotsInFormation{};

KickOffThemFormation::KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
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

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, locations[robotsInFormation.size() - 1]);
}

std::vector<world_new::view::RobotView> KickOffThemFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai