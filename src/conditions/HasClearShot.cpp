/*
 * Returns SUCCESS if the robot has a clear shot to their goal
 * the orientation of the robot is not taken into account.
 * it just draws a line from the robot position towards the goal center and looks for obstacles.
 * otherwise FAILURE
 */

#include "conditions/HasClearShot.h"

#include <coach/PassCoach.h>
#include <control/ControlUtils.h>
#include <world/Ball.h>
#include <world/Field.h>
#include <world/World.h>
#include <world/WorldData.h>

namespace rtt::ai {

HasClearShot::HasClearShot(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

HasClearShot::Status HasClearShot::onUpdate() {
    if ((Vector2(ball->getPos()) - field->get_field().get(THEIR_GOAL_CENTER)).length() < FORCED_SHOOTING_DISTANCE) {
        return Status::Success;
    }

    auto minViewAtGoal = MIN_VIEW_AT_GOAL;
    minViewAtGoal = 0.1;

    // return failure if the robot is too far away for a shot at goal
    if ((Vector2(ball->getPos()) - field->get_field().get(THEIR_GOAL_CENTER)).length() > MAX_SHOOTING_DISTANCE) {
        return Status::Failure;
    }

    if (field->pointIsInDefenceArea(robot->pos, false, 0.5, false)) {
        minViewAtGoal /= 4;
    }

    // return success if there is a clear line to their goal
    bool hasClearShot = field->getPercentageOfGoalVisibleFromPoint(false, ball->getPos(), world->getWorld(), robot->id, true) > minViewAtGoal * 100;

    return hasClearShot ? Status::Success : Status::Failure;
}

}  // namespace rtt::ai
