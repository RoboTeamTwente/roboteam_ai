/*
 * returns SUCCESS if the ball is kicked to the goal. Otherwise FAILURE.
 */
#include <world/Field.h>
#include <world/Ball.h>
#include "conditions/BallKickedToOurGoal.h"
#include "control/ControlUtils.h"

namespace rtt::ai {

BallKickedToOurGoal::BallKickedToOurGoal(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status BallKickedToOurGoal::onUpdate() {

    // Check if the ball is moving at all
    bool ballIsLayingStill = (Vector2(ball->getVel())).length() < Constants::BALL_STILL_VEL();
    if (ballIsLayingStill) { 
        return Status::Failure;
    }

    // determine the goalsides
    Vector2 goalCentre = field->get_field().get(OUR_GOAL_CENTER);
    double goalWidth = field->get_field().get(GOAL_WIDTH);
    double margin = BALL_TO_GOAL_MARGIN;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth/2 + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth/2 + margin);

    // determine the ball position and predicted ball position
    Vector2 ballPos = ball->getPos();
    Vector2 ballPredPos = Vector2(ballPos) + Vector2(ball->getVel()) * BALL_TO_GOAL_TIME;
   
    // Check if the extension of the velocity vector goes through the goal.
    // The line drawn for the ball is the predicted position in 1.5 seconds
    if (control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos)) {
        return Status::Success;
    }
    
    return Status::Failure;
}

}//rtt