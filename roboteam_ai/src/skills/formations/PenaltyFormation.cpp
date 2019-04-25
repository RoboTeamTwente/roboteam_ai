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
        // first we calculate all the positions for the defense
        std::vector<int> robotIds;
        for (auto & i : *robotsInFormation) {
            robotIds.push_back(i->id);
        }
        auto poses = getPenaltyPositions(robotsInFormation->size());

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, poses);
        return shortestDistances.at(robot->id);
    } else {
        //TODO
        return {};
    }
}

shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> rtt::ai::PenaltyFormation::robotsInFormationPtr() {
    return robotsInFormation;
}


std::vector<Vector2> rtt::ai::PenaltyFormation::getPenaltyPositions(int number) {

    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;

    std::vector<Vector2> temp = {{- lengthOffset, widthOffset},
                                 {0, widthOffset},
                                 {lengthOffset, widthOffset},
                                 {lengthOffset, - widthOffset},
                                 {0, - widthOffset},
                                 {- lengthOffset, - widthOffset}};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;

}