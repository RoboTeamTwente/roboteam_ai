/*
 * Determine if it is better that two robots do ballplacement
 * returns SUCCESS if the robot closest to ball is not the same as the robot closest to the target position,
 * EXCEPT if that distance is smaller than MAX_ONE_ROBOT_BALLPLACEMENT_DIST_TO_TARGET.
 */

#include "TwoRobotBallPlacement.h"

namespace rtt {
namespace ai {

TwoRobotBallPlacement::TwoRobotBallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {};

bt::Node::Status TwoRobotBallPlacement::onUpdate() {
    int robotClosestToBallID = world::world->getRobotClosestToBall(OUR_ROBOTS)->id;

    Vector2 ballPlacementPosition = coach::g_ballPlacement.getBallPlacementPos();
    int robotClosestToBallPlacementPosition = world::world->getRobotClosestToPoint(ballPlacementPosition, OUR_ROBOTS)->id;

    bool robotClosestToBallIsClosestToTarget = robotClosestToBallID == robotClosestToBallPlacementPosition;
    bool distanceFromBallToTargetIsSmall =  (ballPlacementPosition - ball->pos).length() < MAX_ONE_ROBOT_BALLPLACEMENT_DIST_TO_TARGET;


    if (!robotClosestToBallIsClosestToTarget && !distanceFromBallToTargetIsSmall) {
        return Status::Success;
    } 
    return Status::Failure;
    
}

}
}
