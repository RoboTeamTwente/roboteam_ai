//
// Created by thijs on 6-7-19.
//

#include "BallPlacementFormation.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "../../control/Hungarian.h"
#include "../../control/ControlUtils.h"

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
    auto formationPositions = getStopPositions().at(amountOfRobots-1);
    std::vector<Vector2> properPositions;
    for (auto pos : formationPositions) {
        if (!positionShouldBeAvoided(pos)) {
            properPositions.push_back(pos);
        }
    }

    int amountToTake = amountOfRobots - properPositions.size();
    std::vector<Vector2> newProposals = getProperPositions(amountToTake);

    for (Vector2 proposal : newProposals) {
        properPositions.push_back(proposal);
    }



    std::vector<int> robotIds;
    for (auto &i : *robotsInFormation) {
        if (robotIds.size() < 8) { // check for amount of robots, we dont want more than 8
            robotIds.push_back(i->id);
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, properPositions);
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

bool BallPlacementFormation::positionShouldBeAvoided(Vector2 pos) {
    Vector2 ballPlacementMarker = rtt::ai::GameStateManager::getRefereeData().designated_position;

    if (!interface::Output::usesRefereeCommands()) {
            ballPlacementMarker = rtt::ai::interface::Output::getInterfaceMarkerPosition();
            std::cerr << "GETTING BALLPLACEMENT LOCATION FROM INTERFACE" << std::endl;
    };
    auto ball = world::world->getBall();
    Vector2 diff = (ball->pos - ballPlacementMarker).rotate(M_PI_2);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->pos + diff.stretchToLength(0.5), ballPlacementMarker + diff.stretchToLength(0.5)}, Qt::magenta, -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->pos - diff.stretchToLength(0.5), ballPlacementMarker - diff.stretchToLength(0.5)}, Qt::magenta, -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->pos, ballPlacementMarker}, Qt::magenta, -1, interface::Drawing::REAL_LIFE_CIRCLES, 0.5, 0.5);

    bool tooCloseToLine = control::ControlUtils::distanceToLineWithEnds(pos, Vector2(ball->pos), ballPlacementMarker) < 0.9;
    return (tooCloseToLine || !world::field->pointIsInField(pos, 0.0));
}

}
}