//
// Created by mrlukasbos on 12-4-19.
//

#include "TimeoutFormation.h"

namespace rtt {
namespace ai {

TimeoutFormation::TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard)
: EnterFormation(name, blackboard) { }

Vector2 TimeoutFormation::getFormationPosition() {
    auto field = world::field->get_field();

    // determine if we should be in the top or bottom of the field
    bool topSideOfField = rtt::ai::interface::InterfaceValues::isTimeOutAtTop();
    int inv = topSideOfField ? 1 : -1;
    double targetLocationY = field.field_width/2 * inv;

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;

    for (unsigned int i = 0; i<robotsInFormation.size(); i++) {
        double targetLocationX = - field.field_length/4 * 2*i*Constants::ROBOT_RADIUS_MAX();
        targetLocations.push_back({targetLocationX, targetLocationY});
        robotIds.push_back(robotsInFormation.at(i)->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
    return shortestDistances.at(robot->id);
}

}
}