//
// Created by rolf on 15-6-19.
//
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include <roboteam_ai/src/world/Field.h>
#include "ResumePlayAfterPenalty.h"
#include "../world/Ball.h"
#include "roboteam_ai/src/control/ControlUtils.h"

namespace rtt{
namespace ai{
ResumePlayAfterPenalty::ResumePlayAfterPenalty(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
}

bt::Node::Status ResumePlayAfterPenalty::onUpdate() {
    bool ballHasMoved= GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->pos) > 0.05;
    bool ballIsLayingStill = (Vector2(ball->vel)).length() < Constants::BALL_STILL_VEL();


    Vector2 goalCentre = world::field->get_our_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    double margin = 0.1;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth/2 + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth/2 + margin);

    // determine the ball position and predicted ball position
    Vector2 ballPos = ball->pos;
    Vector2 ballPredPos = Vector2(ballPos) + Vector2(ball->vel)*Constants::MAX_INTERCEPT_TIME();

    // Check if the extension of the velocity vector goes through the goal.
    // The line drawn for the ball is the predicted position in 1.5 seconds
    bool shotAtGoal=control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos);

    if (!ballShot&&!ballIsLayingStill&&ballHasMoved&&shotAtGoal) {
        ballShot=true;
    }
    if (ballShot){
        ticks++;
    }
    std::cout<<ticks<<std::endl;
    if (ticks>1.5*Constants::TICK_RATE()){
        ticks=0;
        ballShot=false;
        return Status::Success;
    }
    return Status::Failure;
}
}
}