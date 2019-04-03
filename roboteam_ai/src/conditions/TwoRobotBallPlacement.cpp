//
// Created by robzelluf on 3/22/19.
//

#include "TwoRobotBallPlacement.h"

namespace rtt {
namespace ai {

TwoRobotBallPlacement::TwoRobotBallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {};

bt::Node::Status TwoRobotBallPlacement::onUpdate() {
    int robotClosestToBallID = world::world->getRobotClosestToBall(world::OUR_ROBOTS).id;

    Vector2 ballPlacementPosition = coach::g_ballPlacement.getBallPlacementPos();
    int robotClosestToBallPlacementPosition = world::world->getRobotClosestToPoint(ballPlacementPosition, world::OUR_ROBOTS).id;

    bool robotClosestToBallIsClosestToTarget = robotClosestToBallID == robotClosestToBallPlacementPosition;
    bool distanceFromBallToTargetIsSmall = ballPlacementPosition.dist(ball->pos) < MAX_ONE_ROBOT_BALLPLACEMENT_DIST_TO_TARGET;


    if (!robotClosestToBallIsClosestToTarget && !distanceFromBallToTargetIsSmall) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

}
}
