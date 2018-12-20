//
// Created by mrlukasbos on 11-12-18.
//

#include "CanSeeGoal.h"

namespace rtt {
namespace ai {

CanSeeGoal::CanSeeGoal(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { }

CanSeeGoal::Status CanSeeGoal::update() {
    robot = getRobotFromProperties(properties);
    if (! robot) return Status::Waiting;
    int offset = 200;
    Vector2 goalLocation = Field::get_their_goal_center();
    Vector2 myLocation = robot->pos;

    // obstacles can be any robot in the field
    for (auto obstacle : World::getAllRobots()) {
        double distanceToLine = control::ControlUtils::distanceToLine(obstacle.pos, myLocation, goalLocation);
        bool isFurtherThanPoint = myLocation.dist(obstacle.pos) > myLocation.dist(goalLocation);
        bool tooCloseToLine = distanceToLine < offset;

        if (! isFurtherThanPoint && tooCloseToLine) {
            return Status::Failure;
        }
    }
    return Status::Success;
}

} // ai
} // rtt