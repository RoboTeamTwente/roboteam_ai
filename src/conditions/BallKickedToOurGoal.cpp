/*
 * returns SUCCESS if the ball is kicked to the goal. Otherwise FAILURE.
 */
#include "conditions/BallKickedToOurGoal.h"

namespace rtt::ai {

BallKickedToOurGoal::BallKickedToOurGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

bt::Node::Status BallKickedToOurGoal::onUpdate() {
    // Check if the ball is moving at all
    bool ballIsLayingStill = (Vector2(ball->get()->getVelocity())).length() < Constants::BALL_STILL_VEL();
    if (ballIsLayingStill) {
        return Status::Failure;
    }

    // determine the goalsides
    Vector2 goalCentre = (*field).getOurGoalCenter();
    double goalWidth = (*field).getGoalWidth();
    double margin = BALL_TO_GOAL_MARGIN;
    Vector2 lowerPost = goalCentre + Vector2(0.0, -(goalWidth / 2 + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth / 2 + margin);

    // determine the ball position and predicted ball position
    Vector2 ballPos = ball->get()->getPos();
    Vector2 ballPredPos = Vector2(ballPos) + Vector2(ball->get()->getVelocity()) * BALL_TO_GOAL_TIME;

    // Check if the extension of the velocity vector goes through the goal.
    // The line drawn for the ball is the predicted position in 1.5 seconds
    if (control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos)) {
        return Status::Success;
    }

    return Status::Failure;
}

}  // namespace rtt::ai