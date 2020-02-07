//
// Created by mrlukasbos on 12-4-19.
//

#include "skills/formations/TimeoutFormation.h"
#include <world/Field.h>
#include <world/FieldComputations.h>

namespace rtt::ai {
std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> TimeoutFormation::robotsInFormation = nullptr;

TimeoutFormation::TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

Vector2 TimeoutFormation::getFormationPosition() {

    // determine if we should be in the top or bottom of the field
    bool topSideOfField = rtt::ai::interface::Output::isTimeOutAtTop();
    double targetLocationY = topSideOfField ? field->getTopmostY() : field->getBottommostY();

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;

    for (unsigned int i = 0; i<robotsInFormation->size(); i++) {
        double targetLocationX = -field->getFieldLength() / 4 * 2 * i * Constants::ROBOT_RADIUS_MAX();
        targetLocations.emplace_back(targetLocationX, targetLocationY);
    }

    return getOptimalPosition(robot->id, *robotsInFormation, targetLocations);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> TimeoutFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai