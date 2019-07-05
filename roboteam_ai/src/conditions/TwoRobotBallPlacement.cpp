/*
 * Determine if it is better that two robots do ballplacement
 * returns SUCCESS if the robot closest to ball is not the same as the robot closest to the target position,
 * EXCEPT if that distance is smaller than MAX_ONE_ROBOT_BALLPLACEMENT_DIST_TO_TARGET.
 */

#include "TwoRobotBallPlacement.h"

namespace rtt {
namespace ai {

TwoRobotBallPlacement::TwoRobotBallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status TwoRobotBallPlacement::onUpdate() {
    Vector2 ballPlacementPos = coach::g_ballPlacement.getBallPlacementPos();
    auto us = world::world->getUs();

    int minimumRequiredRobotsInField = robotDealer::RobotDealer::keeperExistsInWorld() ? 3 : 2;
    bool weHaveEnoughRobots = us.size() >= minimumRequiredRobotsInField;
    //TODO: THIS REMOVES TWOROBOTBALLPLACEMENT (15.1)
    bool ballIsCloseToBallPlacementPos = ballPlacementPos.dist(ball->pos) < 15.1;


    if (!weHaveEnoughRobots || ballIsCloseToBallPlacementPos) {
        return Status::Failure;
    }

    return Status::Success;
}

}
}
