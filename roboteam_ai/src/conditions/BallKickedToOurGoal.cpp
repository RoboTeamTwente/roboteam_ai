//
// Created by rolf on 12/12/18.
//

#include "BallKickedToOurGoal.h"
namespace rtt {
namespace ai {

BallKickedToOurGoal::BallKickedToOurGoal(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) { };
bt::Node::Status BallKickedToOurGoal::update() {
    roboteam_msgs::WorldBall ball = World::getBall();
    Vector2 goalCentre = Field::get_our_goal_center();
    double goalWidth = Field::get_field().goal_width;
    double margin = constants::BALL_TO_GOAL_MARGIN;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth + margin);
    Vector2 ballPos = ball.pos;
    Vector2 ballPredPos = Vector2(ball.pos) + Vector2(ball.vel)*constants::BALL_TO_GOAL_TIME;
    // Check if the extension of the velocity vector goes through the goal.
    // The line drawn for the ball is the predicted position in 1.5 seconds
    if (control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos)) {
        return Status::Success;
    }
    else {
        return Status::Failure;
    }
}
std::string BallKickedToOurGoal::node_name() {return "BallKickedToOurGoal";}
}//ai
}//rtt