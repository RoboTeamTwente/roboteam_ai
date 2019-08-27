//
// Created by mrlukasbos on 12-4-19.
//

#include <roboteam_ai/src/world/Field.h>
#include "include/roboteam_ai/skills/formations/TimeoutFormation.h"
#include "include/roboteam_ai/control/Hungarian.h"

namespace rtt {
namespace ai {

    std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> TimeoutFormation::robotsInFormation = nullptr;

    TimeoutFormation::TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard)
: Formation(name, blackboard) {
        robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
    }

Vector2 TimeoutFormation::getFormationPosition() {
    auto field = world::field->get_field();

    // determine if we should be in the top or bottom of the field
    bool topSideOfField = rtt::ai::interface::Output::isTimeOutAtTop();
    int inv = topSideOfField ? 1 : -1;
    double targetLocationY = field.field_width/2 * inv;

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;

    for (unsigned int i = 0; i<robotsInFormation->size(); i++) {
        double targetLocationX = - field.field_length/4 * 2*i*Constants::ROBOT_RADIUS_MAX();
        targetLocations.emplace_back(targetLocationX, targetLocationY);
        robotIds.push_back(robotsInFormation->at(i)->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
    return shortestDistances.at(robot->id);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> TimeoutFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

}
}