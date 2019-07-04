
#include "StopFormation.h"
#include <roboteam_ai/src/world/Field.h>
#include "../../control/Hungarian.h"

namespace rtt {
namespace ai {

std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> StopFormation::robotsInFormation = nullptr;

StopFormation::StopFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

Vector2 StopFormation::getFormationPosition() {
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

std::shared_ptr<std::vector<world::World::RobotPtr>> StopFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

}
}