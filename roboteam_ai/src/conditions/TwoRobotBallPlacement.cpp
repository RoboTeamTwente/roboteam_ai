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
    Vector2 ballPlacementPos = coach::g_ballPlacement.getBallPlacementPos();
    Vector2 ballPos = ball->pos;
    auto us = world::world->getUs();
    if (us.size() < 2) {
        return Status::Failure;
    }

    if ((ball->pos - ballPlacementPos).length() < 2.1) {
        return Status::Failure;
    } else {
        return Status::Success;
    }
}

}
}
