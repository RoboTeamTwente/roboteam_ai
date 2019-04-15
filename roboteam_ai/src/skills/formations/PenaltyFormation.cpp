//
// Created by baris on 15-4-19.
//

#include "PenaltyFormation.h"
rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {

}
Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    if(properties->getBool("Offensive")) {
        auto field = world::field->get_field();

        double targetLocationX = field.field_length/5;

        // first we calculate all the positions for the defense
        std::vector<Vector2> targetLocations;
        std::vector<int> robotIds;

        for (unsigned int i = 0; i < robotsInFormation.size(); i ++) {

            double targetLocationY = - field.field_width/4*2*i*Constants::ROBOT_RADIUS_MAX();
            targetLocations.emplace_back(targetLocationX, targetLocationY);

            robotIds.push_back(robotsInFormation.at(i)->id);
        }

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
        return shortestDistances.at(robot->id);
    } else {
        return {};
    }
}
