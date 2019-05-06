//
// Created by rolf on 12/12/18.
//

#include "InterceptBall.h"
#include "roboteam_ai/src/interface/api/Input.h"
#include "roboteam_ai/src/world/Field.h"

namespace rtt {
namespace ai {

InterceptBall::InterceptBall(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { };

//TODO: make prediction for the Robot if it can even intercept the ball at all from the initialization state.
void InterceptBall::onInitialize() {
    keeper = properties->getBool("Keeper");

    currentProgression = INTERCEPTING;

    tickCount = 0;
    maxTicks = static_cast<int>(floor(Constants::MAX_INTERCEPT_TIME() * Constants::TICK_RATE()));
    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();

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
}
InterceptBall::Status InterceptBall::onUpdate() {
    ball = world::world->getBall();
    //The keeper dynamically updates the intercept position as he needs to be responsive and cover the whole goal and this would help against curveballs etc.
    if (keeper) {
        interceptPos = computeInterceptPoint(ball->pos,
                Vector2(ball->pos) + Vector2(ball->vel) * Constants::MAX_INTERCEPT_TIME());
    }
    deltaPos = interceptPos - robot->pos;
    checkProgression();
    //interface
    displayColorData.emplace_back(std::make_pair(interceptPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ballStartPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ballEndPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(ball->pos,Qt::green));
    displayColorData.emplace_back(std::make_pair(Vector2(ball->pos)+ Vector2(ball->vel) * Constants::MAX_INTERCEPT_TIME(),Qt::green));
    interface::Input::setInterceptPoints(robot->id,displayColorData);
    displayColorData.clear();


    tickCount ++;
    switch (currentProgression) {
        case INTERCEPTING:
            sendInterceptCommand();
            //sendMoveCommand(interceptPos);
            return Status::Running;
        case CLOSETOPOINT:
            sendFineInterceptCommand();
            //sendMoveCommand(interceptPos);
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

void InterceptBall::sendMoveCommand(Vector2 targetPos) {
    Vector2 velocities = numtreeGTP.getPosVelAngle(robot, targetPos).vel;
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);

    publishRobotCommand();
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
        if (dist < Constants::ROBOT_RADIUS()) {
            currentProgression = CLOSETOPOINT;
        };//If robot is close, switch to closetoPoint
        return;
    case CLOSETOPOINT:
        if (dist < INTERCEPT_POSDIF) {
            currentProgression = INPOSITION;
        }//If Robot overshoots, switch to overshoot, if in Position, go there
        else if (dist >= Constants::ROBOT_RADIUS()) {
            currentProgression = INTERCEPTING;
        }
        return;
    case INPOSITION:
        if (dist < INTERCEPT_POSDIF) {
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
        // Depends on two keeper Constants in Constants!
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
    double angleDev = tan(Constants::ROBOT_RADIUS()/interceptDist);
    double rectHalfLength = atan(angleDev)
            *(interceptDist + Constants::ROBOT_RADIUS());// Half of the rectangle covered by the robot behind the robot.
    // We check if the ball ever comes into the rectangle behind the robot that it should 'cover off'
    Vector2 rectSide = ballVel.rotate(M_PI_2).stretchToLength(rectHalfLength);
    Vector2 startCentre = startBall + ballVel.stretchToLength(interceptDist + Constants::ROBOT_RADIUS());
    Vector2 endCentre = startBall + (endBall - startBall)*2; //twice the predicted length to be sure
    return control::ControlUtils::pointInRectangle(ball->pos, startCentre - rectSide, startCentre + rectSide,
            endCentre + rectSide, endCentre - rectSide);
}

//Checks if the ball was deflected by the Robot
bool InterceptBall::ballDeflected() {
    // A ball is deflected if:
    // If ball velocity changes by more than x degrees from the original orientation then it is deflected
    if (abs(control::ControlUtils::constrainAngle(Vector2(ball->vel).angle() - ballStartVel.angle()))
            > BALL_DEFLECTION_ANGLE) {
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
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = static_cast<float>((ballStartPos-interceptPos).angle()); //Rotates orthogonal to the line of the ball
    publishRobotCommand();
}

void InterceptBall::sendFineInterceptCommand() {
    auto pva = basicGTP.getPosVelAngle(robot, interceptPos);

    command.x_vel = pva.vel.x;
    command.y_vel = pva.vel.y;
    command.w = static_cast<float>((Vector2(ball->pos)-Vector2(robot->pos)).angle()); //Rotates towards the ball
    publishRobotCommand();
}
void InterceptBall::sendInterceptCommand() {
    auto pva = numtreeGTP.getPosVelAngle(robot, interceptPos);

    command.x_vel = pva.vel.x;
    command.y_vel = pva.vel.y;
    if (backwards) {
        command.w = pva.angle.getAngle() + M_PI;
    }
    else{
        command.w = pva.angle.getAngle();
    }
    publishRobotCommand();

}
//Checks if the ball is kicked to Goal. Kind of duplicate to the condition, but this uses an extra saftey margin
bool InterceptBall::ballToGoal() {
    Vector2 goalCentre = world::field->get_our_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    double margin = Constants::BALL_RADIUS();
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth + margin));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth + margin);
    Vector2 ballPos = ball->pos;
    Vector2 ballPredPos = Vector2(ball->pos) + Vector2(ball->vel) * Constants::MAX_INTERCEPT_TIME();
    return control::ControlUtils::lineSegmentsIntersect(lowerPost, upperPost, ballPos, ballPredPos);
}
// Checks if the ball is in our Goal (e.g. the opponent scored)
bool InterceptBall::ballInGoal() {
    Vector2 goalCentre = world::field->get_our_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth);
    Vector2 depth = Vector2(- world::field->get_field().goal_depth, 0.0);
    return control::ControlUtils::pointInRectangle(ball->pos, lowerPost, lowerPost + depth, upperPost + depth, upperPost);
}

}//ai
}//rtt