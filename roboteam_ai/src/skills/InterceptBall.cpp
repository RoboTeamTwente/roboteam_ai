//
// Created by rolf on 12/12/18.
//

#include "InterceptBall.h"

namespace rtt {
namespace ai {

InterceptBall::InterceptBall(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { };

//TODO: make prediction for the Robot if it can even intercept the ball at all from the initialization state.
void InterceptBall::onInitialize() {
    keeper = properties->getBool("Keeper");

    currentProgression = INTERCEPTING;

    tickCount = 0;
    maxTicks = static_cast<int>(floor(constants::MAX_INTERCEPT_TIME*constants::tickRate));
    ball = World::getBall();
    ballStartPos = ball.pos;
    ballStartVel = ball.vel;
    ballEndPos = Vector2(ball.pos) + Vector2(ball.vel)*constants::MAX_INTERCEPT_TIME;
    if (robot) interceptPos = computeInterceptPoint(ballStartPos, ballEndPos);
    else currentProgression = BALLMISSED;
    pid.setParams(4.0, 0.0, 0.75, 10, 0.0, 0.0); //TODO: magic numbers galore, from the old team. Move to new control library?
    finePid.setParams(1.0, 0.0, 0.0, 0, 0.0, 0.0);
    pid.initialize(1.0/constants::tickRate);
    finePid.initialize(1.0/constants::tickRate);
}
InterceptBall::Status InterceptBall::onUpdate() {
    ball = World::getBall();
    //The keeper dynamically updates the intercept position as he needs to be responsive and cover the whole goal and this would help against curveballs e.g.
    if (keeper) {
        interceptPos = computeInterceptPoint(ball.pos,
                Vector2(ball.pos) + Vector2(ball.vel)*constants::MAX_INTERCEPT_TIME);
    }
    deltaPos = interceptPos - robot->pos;
    checkProgression();
    tickCount ++;
    switch (currentProgression) {
        case INTERCEPTING:
            sendInterceptCommand();
            return Status::Running;
        case CLOSETOPOINT:
            sendFineInterceptCommand();
            return Status::Running;
        case OVERSHOOT:
            sendInterceptCommand();
            return Status::Running;
        case INPOSITION:
            sendStopCommand();
            return Status::Running;
        case BALLDEFLECTED:
            return Status::Success;
        case BALLMISSED:
            return Status::Failure;
    }

    return Status::Failure;
}

void InterceptBall::checkProgression() {
    if (keeper) {
        if (ballInGoal()) {
            currentProgression = BALLMISSED;
        }
        //Check if the ball was deflected
        if (! ballToGoal()) {
            currentProgression = BALLDEFLECTED;
            return;
        }
        //
    }
    else {
        //check if we missed the ball
        if (missBall(ballStartPos, ballEndPos, ballStartVel) || tickCount > maxTicks) {
            currentProgression = BALLMISSED;
            return;
        }
            //check if ball is deflected
        else if (ballDeflected()) {
            currentProgression = BALLDEFLECTED;
            return;
        }
    }

    double dist = deltaPos.length();
    //Update the state of the robot
    switch (currentProgression) {
        case INTERCEPTING:
            if (dist < 2*constants::ROBOT_RADIUS) {
                currentProgression = CLOSETOPOINT;
            };//If robot is close, switch to closetoPoint
            return;
        case CLOSETOPOINT:
            if (dist < constants::INTERCEPT_POSDIF) {
                currentProgression = INPOSITION;
            }//If Robot overshoots, switch to overshoot, if in Position, go there
            else if (dist >= 2*constants::ROBOT_RADIUS) {
                currentProgression = OVERSHOOT;
            }
            return;
        case OVERSHOOT:
            if (dist < 2*constants::ROBOT_RADIUS) {
                currentProgression = CLOSETOPOINT;
            };// Go back to closetopoint
        case INPOSITION:
            if (dist < constants::INTERCEPT_POSDIF) {
                return;
            }
            else {
                currentProgression = CLOSETOPOINT;
                return;
            }// Stay here until either ball misses or is deflected;
        case BALLDEFLECTED:
            return;
        case BALLMISSED:
            return;
    }

};
void InterceptBall::onTerminate(rtt::ai::Skill::Status s) {
    sendStopCommand();
}

Vector2 InterceptBall::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;
    if (keeper) {
        Arc keeperCircle = control::ControlUtils::createKeeperArc();
        std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = keeperCircle.intersectionWithLine(
                startBall, endBall);
        if (intersections.first && intersections.second) {
            double dist1 = (Vector2(robot->pos) - *intersections.first).length();
            double dist2 = (Vector2(robot->pos) - *intersections.second).length();
            if (dist2 < dist1) {
                interceptionPoint = *intersections.second;
            }
            else { interceptionPoint = *intersections.first; }
        }
        else if (intersections.first) {
            interceptionPoint = *intersections.first;
        }
        else if (intersections.second) {
            interceptionPoint = *intersections.second;
        }
        else { // if the Line does not intercept it usually means the ball is coming from one of the corners-ish to the keeper
            interceptionPoint = Vector2(robot->pos).project(startBall,
                    endBall); // For now we pick the closest point to the (predicted) line of the ball
        }
    }
    else {
        interceptionPoint = Vector2(robot->pos).project(startBall,
                endBall); // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
    }
    return interceptionPoint;
}
// Checks if the Robot already missed the Ball
bool InterceptBall::missBall(Vector2 startBall, Vector2 endBall, Vector2 ballVel) {
    double interceptDist = (interceptPos - startBall).length();
    double angleDev = tan(constants::ROBOT_RADIUS/interceptDist);
    double rectHalfLength = atan(angleDev)
            *(interceptDist + constants::ROBOT_RADIUS);// Half of the rectangle covered by the robot behind the robot.
    // We check if the ball ever comes into the rectangle behind the robot that it should 'cover off'
    Vector2 rectSide = ballVel.rotate(M_PI_2).stretchToLength(rectHalfLength);
    Vector2 startCentre = startBall + ballVel.stretchToLength(interceptDist + constants::ROBOT_RADIUS);
    Vector2 endCentre = startBall + (endBall - startBall)*2; //twice the predicted length to be sure
    return control::ControlUtils::pointInRectangle(ball.pos, startCentre - rectSide, startCentre + rectSide,
            endCentre + rectSide, endCentre - rectSide);
}

//Checks if the ball was deflected by the Robot
bool InterceptBall::ballDeflected() {
    // A ball is deflected if:
    // If ball velocity changes by more than x degrees from the original orientation then it is deflected
    if (abs(control::ControlUtils::constrainAngle(Vector2(ball.vel).angle() - ballStartVel.angle()))
            > constants::BALL_DEFLECTION_ANGLE) {
        return true;
    }
    // Ball Position is behind the line orthogonal to the ball velocity going through the ballStartPos
    Vector2 lineEnd = ballStartPos + ballStartVel.rotate(M_PI_2);
    // https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
    double ballEndPosSide = (ballEndPos.x - ballStartPos.x)*(lineEnd.y - ballStartPos.y)
            - (ballEndPos.y - ballStartPos.y)*(lineEnd.x - ballStartPos.x);
    double ballPosSide = (ball.pos.x - ballStartPos.x)*(lineEnd.y - ballStartPos.y)
            - (ball.pos.y - ballStartPos.y)*(lineEnd.x - ballStartPos.x);
    if (ballEndPosSide < 0) {
        return ballPosSide > 0;
    }
    if (ballEndPosSide > 0) {
        return ballPosSide < 0;
    }
    return true;
}
void InterceptBall::sendStopCommand() {
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robotId;
    cmd.x_vel = 0;
    cmd.y_vel = 0;
    cmd.w = static_cast<float>(M_PI_2);// TODO: CHange this to rotate towards the ball
    publishRobotCommand(cmd);
}
void InterceptBall::sendFineInterceptCommand() {
    Vector2 delta = pid.posControl(robot->pos, interceptPos);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(delta.x);
    cmd.y_vel = static_cast<float>(delta.y);
    cmd.w = static_cast<float>(M_PI_2); //TODO: Fix angles
    publishRobotCommand(cmd);
}
void InterceptBall::sendInterceptCommand() {
    Vector2 delta = finePid.posControl(robot->pos, interceptPos);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(delta.x);
    cmd.y_vel = static_cast<float>(delta.y);
    cmd.w = static_cast<float>(M_PI_2);// TODO: Fix angles
    publishRobotCommand(cmd);

}
bool InterceptBall::ballToGoal() {
    Vector2 goalCentre = Field::get_our_goal_center();
    double goalWidth = Field::get_field().goal_width;
    double margin = constants::BALL_TO_GOAL_MARGIN;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth + margin);
    Vector2 ballPos = ball.pos;
    Vector2 ballPredPos = Vector2(ball.pos) + Vector2(ball.vel)*constants::MAX_INTERCEPT_TIME;
    return control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos);
}
bool InterceptBall::ballInGoal() {
    Vector2 goalCentre = Field::get_our_goal_center();
    double goalWidth = Field::get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth);
    Vector2 depth = Vector2(- Field::get_field().goal_depth, 0.0);
    return control::ControlUtils::pointInRectangle(ball.pos, lowerPost, lowerPost + depth, upperPost + depth,
            upperPost);
}
}//ai
}//rtt