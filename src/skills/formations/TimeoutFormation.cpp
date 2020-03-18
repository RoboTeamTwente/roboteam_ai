//
// Created by mrlukasbos on 12-4-19.
//

#include <skills/formations/TimeoutFormation.h>

namespace rtt::ai {
std::vector<world_new::view::RobotView> TimeoutFormation::robotsInFormation{};

TimeoutFormation::TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(std::move(name), std::move(blackboard)) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
}

Vector2 TimeoutFormation::getFormationPosition() {

    // determine if we should be in the top or bottom of the field
    bool topSideOfField = rtt::ai::interface::Output::isTimeOutAtTop();
    double targetLocationY = topSideOfField ? field->getTopmostY() : field->getBottommostY();

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;

    for (unsigned int i = 0; i < robotsInFormation.size(); i++) {
        double targetLocationX = -field->getFieldLength() / 4 * 2 * i * Constants::ROBOT_RADIUS_MAX();
        targetLocations.emplace_back(targetLocationX, targetLocationY);
    }

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, targetLocations);
}

std::vector<world_new::view::RobotView> TimeoutFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai