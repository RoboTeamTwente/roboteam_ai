/*
 * Returns SUCCESS if the robot has a clear shot to their goal
 * the orientation of the robot is not taken into account. 
 * it just draws a line from the robot position towards the goal center and looks for obstacles.
 * otherwise FAILURE
 */


#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/coach/PassCoach.h>
#include "HasClearShot.h"

namespace rtt{
namespace ai {

HasClearShot::HasClearShot(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {}

HasClearShot::Status HasClearShot::onUpdate() {
    if ((Vector2(ball->pos) - world::field->get_their_goal_center()).length() < 2.5) {
        return Status::Success;
    }

    auto minViewAtGoal = MIN_VIEW_AT_GOAL;
    minViewAtGoal = 0.2;

	// return failure if the robot is too far away for a shot at goal
    if ((Vector2(ball->pos) - world::field->get_their_goal_center()).length() > MAX_SHOOTING_DISTANCE) {
        return Status::Failure;
    }

    if (world::field->pointIsInDefenceArea(robot->pos, false, 0.5, false)) {
        minViewAtGoal /= 4;
    }

    // return success if there is a clear line to their goal 
    auto world = world::world->getWorld();
    bool hasClearShot = world::field->getPercentageOfGoalVisibleFromPoint(false, ball->pos, world, robot->id, true) >
            minViewAtGoal * 100;

    return hasClearShot ? Status::Success : Status::Failure;
}

} // ai
} // rtt
