//
// Created by thijs on 6-7-19.
//

#include "BallPlacementFormation.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "../../control/Hungarian.h"

namespace rtt {
namespace ai {

std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> BallPlacementFormation::robotsInFormation = nullptr;


BallPlacementFormation::BallPlacementFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : StopFormation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

Vector2 BallPlacementFormation::getFormationPosition() {

    //failsafe to prevent segfaults
    int amountOfRobots = robotsInFormation->size();
    if (amountOfRobots <= 0) {
        return {};
    }

    std::vector<int> robotIds;
    for (auto &i : *robotsInFormation) {
        if (robotIds.size() < 8) { // check for amount of robots, we dont want more than 8
            robotIds.push_back(i->id);
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, getStopPositions().at(amountOfRobots - 1));
    return shortestDistances.at(robot->id);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> BallPlacementFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

// adapt to the change of robot amount in formation
void BallPlacementFormation::updateFormation() {
    targetLocation = getFormationPosition();
    robotsInFormationMemory = robotsInFormationPtr()->size();
}

}
}