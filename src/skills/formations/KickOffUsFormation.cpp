#include <skills/formations/KickOffUsFormation.h>

namespace rtt::ai {
std::vector<world_new::view::RobotView> KickOffUsFormation::robotsInFormation{};

KickOffUsFormation::KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(std::move(name), std::move(blackboard)) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
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

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, locations[robotsInFormation.size() - 1]);
}

std::vector<world_new::view::RobotView> KickOffUsFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai