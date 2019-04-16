//
// Created by baris on 15-4-19.
//

#include "PenaltyFormation.h"

std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::PenaltyFormation::robotsInFormation = nullptr;

rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    if(properties->getBool("Offensive")) {
        auto field = world::field->get_field();

        double targetLocationX = field.field_length/4;

        // first we calculate all the positions for the defense
        std::vector<Vector2> targetLocations;
        std::vector<int> robotIds;

        for (unsigned int i = 0; i < robotsInFormation->size(); i ++) {

            // beautiful magic that works
            double targetLocationY = - field.field_width*i*Constants::ROBOT_RADIUS_MAX() + field.field_width/4;
            targetLocations.emplace_back(targetLocationX, targetLocationY);

            robotIds.push_back(robotsInFormation->at(i)->id);
        }

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
        return shortestDistances.at(robot->id);
    } else {
        return {};
    }
}

shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> rtt::ai::PenaltyFormation::robotsInFormationPtr() {
    return robotsInFormation;
}
