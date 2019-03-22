//
// Created by robzelluf on 3/22/19.
//

#include "TwoRobotBallPlacement.h"

namespace rtt {
namespace ai {

TwoRobotBallPlacement::TwoRobotBallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {};

bt::Node::Status TwoRobotBallPlacement::onUpdate() {
    int robotClosestToBallID = World::getRobotClosestToBall(true)->id;

    Vector2 ballPlacementPosition = coach::g_ballPlacement.getBallPlacementPos();
    int robotClosestToBallPlacementPosition = World::getRobotClosestToPoint(ballPlacementPosition, true)->id;

    if (robotClosestToBallID != robotClosestToBallPlacementPosition) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

}
}
