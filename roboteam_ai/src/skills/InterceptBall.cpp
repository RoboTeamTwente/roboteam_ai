//
// Created by rolf on 12/12/18.
//

#include "InterceptBall.h"
#include "../interface/drawer.h"
#include "../utilities/Field.h"

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
    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * constants::MAX_INTERCEPT_TIME;

    if (robot) {
        interceptPos = computeInterceptPoint(ballStartPos,ballEndPos);
        deltaPos=interceptPos-robot->pos;
        // Checks if it is faster to go to the interceptPos backwards or forwards (just which is closer to the current orientation)
        backwards=control::ControlUtils::angleDifference(robot->angle,deltaPos.angle())>M_PI_2;
    }
    else {
        currentProgression = BALLMISSED;
        backwards=false;
    }
    pid.setPID(5.7,1.7,0.0,1.0/constants::tickRate); //TODO:magic numbers galore, from the old team. Move to new control library?
    finePid.setPID(5.7,1.7,0.0, 1.0/constants::tickRate);
}
InterceptBall::Status InterceptBall::onUpdate() {
    ball = World::getBall();
    //The keeper dynamically updates the intercept position as he needs to be responsive and cover the whole goal and this would help against curveballs etc.
    if (keeper) {
        interceptPos = computeInterceptPoint(ball->pos,
                Vector2(ball->pos) + Vector2(ball->vel)*constants::MAX_INTERCEPT_TIME);
    }
    deltaPos = interceptPos - robot->pos;
    checkProgression();
    //interface
    displayColorData.emplace_back(std::make_pair(interceptPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ballStartPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ballEndPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ball->pos,Qt::green));
    displayColorData.emplace_back(std::make_pair(Vector2(ball->pos)+ Vector2(ball->vel)*constants::MAX_INTERCEPT_TIME,Qt::green));
    interface::Drawer::setInterceptPoints(robot->id,displayColorData);
    displayColorData.clear();


    tickCount ++;
    switch (currentProgression) {
        case INTERCEPTING:
            sendInterceptCommand();
            return Status::Running;
        case CLOSETOPOINT:
            sendFineInterceptCommand();
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
        if (ballInGoal()|| missedBall(ballStartPos, ballEndPos, ballStartVel)) {
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
        if (missedBall(ballStartPos, ballEndPos, ballStartVel) || tickCount > maxTicks) {
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
        if (dist < constants::ROBOT_RADIUS) {
            currentProgression = CLOSETOPOINT;
        };//If robot is close, switch to closetoPoint
        return;
    case CLOSETOPOINT:
        if (dist < constants::INTERCEPT_POSDIF) {
            currentProgression = INPOSITION;
        }//If Robot overshoots, switch to overshoot, if in Position, go there
        else if (dist >= constants::ROBOT_RADIUS) {
            currentProgression = INTERCEPTING;
        }
        return;
    case INPOSITION:
        if (dist < constants::INTERCEPT_POSDIF) {
            return;
        }
        else {
            currentProgression = CLOSETOPOINT;
            return;
        }// Stay here until either ball misses or is deflected;
    case BALLDEFLECTED: return;
    case BALLMISSED: return;
    }

};
void InterceptBall::onTerminate(rtt::ai::Skill::Status s) {
    sendStopCommand();
}

Vector2 InterceptBall::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;
    if (keeper) {
        //This is done in control library as it is needed in intercept too
        // Depends on two keeper constants in constants!
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
        else {
            // if the Line does not intercept it usually means the ball is coming from one of the corners-ish to the keeper
            // For now we pick the closest point to the (predicted) line of the ball
            interceptionPoint = Vector2(robot->pos).project(startBall, endBall); }
    }
    else {
        // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
        interceptionPoint = Vector2(robot->pos).project(startBall,endBall);
    }
    return interceptionPoint;
}
// Checks if the Robot already missed the Ball
bool InterceptBall::missedBall(Vector2 startBall, Vector2 endBall, Vector2 ballVel) {
    double interceptDist = (interceptPos - startBall).length();
    double angleDev = tan(constants::ROBOT_RADIUS/interceptDist);
    double rectHalfLength = atan(angleDev)
            *(interceptDist + constants::ROBOT_RADIUS);// Half of the rectangle covered by the robot behind the robot.
    // We check if the ball ever comes into the rectangle behind the robot that it should 'cover off'
    Vector2 rectSide = ballVel.rotate(M_PI_2).stretchToLength(rectHalfLength);
    Vector2 startCentre = startBall + ballVel.stretchToLength(interceptDist + constants::ROBOT_RADIUS);
    Vector2 endCentre = startBall + (endBall - startBall)*2; //twice the predicted length to be sure
    return control::ControlUtils::pointInRectangle(ball->pos, startCentre - rectSide, startCentre + rectSide,
            endCentre + rectSide, endCentre - rectSide);
}

//Checks if the ball was deflected by the Robot
bool InterceptBall::ballDeflected() {
    // A ball is deflected if:
    // If ball velocity changes by more than x degrees from the original orientation then it is deflected
    if (abs(control::ControlUtils::constrainAngle(Vector2(ball->vel).angle() - ballStartVel.angle()))
            > constants::BALL_DEFLECTION_ANGLE) {
        return true;
    }
    // Ball Position is behind the line orthogonal to the ball velocity going through the ballStartPos
    Vector2 lineEnd = ballStartPos + ballStartVel.rotate(M_PI_2);
    // https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
    double ballEndPosSide = (ballEndPos.x - ballStartPos.x)*(lineEnd.y - ballStartPos.y)
            - (ballEndPos.y - ballStartPos.y)*(lineEnd.x - ballStartPos.x);
    double ballPosSide = (ball->pos.x - ballStartPos.x)*(lineEnd.y - ballStartPos.y)
            - (ball->pos.y - ballStartPos.y)*(lineEnd.x - ballStartPos.x);
    if (ballEndPosSide < 0) {
        return ballPosSide > 0;
    }
    if (ballEndPosSide > 0) {
        return ballPosSide < 0;
    }
    return true;
}
void InterceptBall::sendStopCommand() {
    roboteam_msgs::RobotCommand command;
    command.use_angle = 1;
    command.id = robotId;
    command.x_vel = 0;
    command.y_vel = 0;
    //TODO: Perhaps make the desired end orientation a boolean/switch?
    command.w = static_cast<float>((ballStartPos-interceptPos).angle()); //Rotates orthogonal to the line of the ball
    publishRobotCommand(command);
}

void InterceptBall::sendFineInterceptCommand() {
    Vector2 error= interceptPos-robot->pos;
    Vector2 delta = pid.controlPIR(error, robot->vel);
    Vector2 deltaLim=control::ControlUtils::VelocityLimiter(delta);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(deltaLim.x);
    cmd.y_vel = static_cast<float>(deltaLim.y);
    cmd.w = static_cast<float>((Vector2(ball->pos)-Vector2(robot->pos)).angle()); //Rotates towards the ball
    publishRobotCommand(cmd);
}
void InterceptBall::sendInterceptCommand() {
    Vector2 delta = finePid.controlPID(interceptPos - robot->pos);
    Vector2 deltaLim=control::ControlUtils::VelocityLimiter(delta);
    roboteam_msgs::RobotCommand command;
    command.use_angle = 1;
    command.id = robot->id;
    command.x_vel = static_cast<float>(deltaLim.x);
    command.y_vel = static_cast<float>(deltaLim.y);
    if (backwards) {
        command.w = static_cast<float>(deltaLim.rotate(M_PI).angle());
    }
    else{
        command.w= static_cast<float>(deltaLim.angle());
    }
    publishRobotCommand(command);

}
//Checks if the ball is kicked to Goal. Kind of duplicate to the condition, but this uses an extra saftey margin
bool InterceptBall::ballToGoal() {
    Vector2 goalCentre = Field::get_our_goal_center();
    double goalWidth = Field::get_field().goal_width;
    double margin = constants::BALL_TO_GOAL_MARGIN;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth + margin);
    Vector2 ballPos = ball->pos;
    Vector2 ballPredPos = Vector2(ball->pos) + Vector2(ball->vel)*constants::MAX_INTERCEPT_TIME;
    return control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos);
}
// Checks if the ball is in our Goal (e.g. the opponent scored)
bool InterceptBall::ballInGoal() {
    Vector2 goalCentre = Field::get_our_goal_center();
    double goalWidth = Field::get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth);
    Vector2 depth = Vector2(- Field::get_field().goal_depth, 0.0);
    return control::ControlUtils::pointInRectangle(ball->pos, lowerPost, lowerPost + depth, upperPost + depth, upperPost);
}

}//ai
}//rtt