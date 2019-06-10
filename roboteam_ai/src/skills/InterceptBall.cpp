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

//TODO: make prediction for the RobotPtr if it can even intercept the ball at all from the initialization state.
void InterceptBall::onInitialize() {
    keeper = properties->getBool("Keeper");
    if (keeper) {
        /// This function is hacky; we need to manually update the PID now everytime.
        poscontroller.setAutoListenToInterface(false);
        poscontroller.updatePid(Constants::standardKeeperInterceptPID());
    }
    currentProgression = INTERCEPTING;
    tickCount = 0;
    maxTicks = static_cast<int>(floor(Constants::MAX_INTERCEPT_TIME()*Constants::TICK_RATE()));
    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel*Constants::MAX_INTERCEPT_TIME();
    if (robot) {
        interceptPos = computeInterceptPoint(ballStartPos, ballEndPos);
        deltaPos = interceptPos - robot->pos;
        // Checks if it is faster to go to the interceptPos backwards or forwards (just which is closer to the current orientation)
        backwards = control::ControlUtils::angleDifference(robot->angle, deltaPos.angle()) > M_PI_2;
    }
    else {
        currentProgression = BALLMISSED;
        backwards = false;
    }
}
InterceptBall::Status InterceptBall::onUpdate() {
    ball = world::world->getBall();
    //The keeper dynamically updates the intercept position as he needs to be responsive and cover the whole goal and this would help against curveballs etc.
    interceptPos = computeInterceptPoint(ball->pos,
            Vector2(ball->pos) + Vector2(ball->vel)*Constants::MAX_INTERCEPT_TIME());

    deltaPos = interceptPos - robot->pos;
    checkProgression();

    interface::Input::drawData(interface::Visual::INTERCEPT, {ballStartPos, ballEndPos}, Qt::darkCyan, robot->id,
            interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::INTERCEPT, {interceptPos}, Qt::cyan, robot->id,
            interface::Drawing::DOTS, 5, 5);
    // if we are not already rotating
    if (deltaPos.length() < TURNING_DISTANCE && ! orientationLocked) {
        // update if we want to rotate or not; if we have time to turn we do so, otherwise not.
        // this assumes we are at 90 to 180 degrees difference with the target angle; can be improved by measuring average turn time under keeper circumstances
        stayAtOrientation = ball->vel.length()*TURN_TIME > (interceptPos - ball->pos).length();
        orientationLocked = true;
    }
    tickCount ++;
    switch (currentProgression) {
    case INTERCEPTING:sendMoveCommand(interceptPos);
        return Status::Running;
    case INPOSITION:sendStopCommand();
        return Status::Running;
    case BALLDEFLECTED:return Status::Success;
    case BALLMISSED:return Status::Failure;
    }

    return Status::Failure;
}

void InterceptBall::sendMoveCommand(Vector2 targetPos) {
    Vector2 velocities;
    if (keeper) {
        /// Manual PID value update. Ugly and should be refactored in the future.
        poscontroller.updatePid(interface::Output::getKeeperInterceptPid());
        velocities = poscontroller.getRobotCommand(robot, targetPos).vel;
    }
    else {
        velocities = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos).vel;
    }
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);

    auto blockAngle = Angle((interceptPos - robot->pos).angle());
    command.w = ! backwards ? blockAngle.getAngle() : Angle(blockAngle + M_PI).getAngle();
    if (orientationLocked) {
        if (stayAtOrientation){
            command.w=Angle((ballStartPos - interceptPos).angle() + M_PI_2).getAngle();
        }
        else {
            command.w = (ballStartPos - interceptPos).angle();
            command.dribbler = 25;
        }
    }
    publishRobotCommand();
}

void InterceptBall::checkProgression() {
    if (keeper) {
        if (ballInGoal() || missedBall(ballStartPos, ballEndPos, ballStartVel)) {
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
        if (dist < INTERCEPT_POSDIF) {
            currentProgression = INPOSITION;
        }
    case INPOSITION:
        if (dist < INTERCEPT_POSDIF) {
            return;
        }
        else {
            currentProgression = INTERCEPTING;
            return;
        }// Stay here until either ball misses or is deflected;
    case BALLDEFLECTED: return;
    case BALLMISSED: return;
    }

};
void InterceptBall::onTerminate(rtt::ai::Skill::Status s) {
    sendStopCommand();
    tickCount = 0;
    currentProgression = INTERCEPTING;
}

Vector2 InterceptBall::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;
    if (keeper) {
        Line shotLine(startBall, endBall);
        interceptionPoint = shotLine.project(robot->pos);
        //create an area in which the intersection point should be
        auto DefenceArea = world::field->getDefenseArea(true);
        if (! DefenceArea.contains(interceptionPoint)) {
            auto intersectPoints = DefenceArea.intersections(
                    LineSegment(shotLine.start, shotLine.start + (shotLine.end - shotLine.start).scale(1000)));
            if (intersectPoints.empty()) {
                return interceptionPoint;
            }
            interceptionPoint = intersectPoints[0];
            double bestDist = (robot->pos - interceptionPoint).length();
            for (int j = 1; j < intersectPoints.size(); ++ j) {
                double dist = (intersectPoints[j] - robot->pos).length();
                if (dist < bestDist) {
                    interceptionPoint = intersectPoints[j];
                    bestDist = dist;
                }
            }
        }
    }
    else {
        // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
        Line shotLine(startBall, endBall);
        interceptionPoint = shotLine.project(robot->pos);
    }
    return interceptionPoint;
}
// Checks if the RobotPtr already missed the BallPtr
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

//Checks if the ball was deflected by the RobotPtr
bool InterceptBall::ballDeflected() {
    // A ball is deflected if:
    // If ball velocity changes by more than x degrees from the original orientation then it is deflected
    if (abs(control::ControlUtils::constrainAngle(Vector2(ball->vel).angle() - ballStartVel.angle()))
            > BALL_DEFLECTION_ANGLE) {
        return true;
    }
    // BallPtr Position is behind the line orthogonal to the ball velocity going through the ballStartPos
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
    if (! stayAtOrientation) {
        command.w = static_cast<float>((ballStartPos
                - interceptPos).angle()); //Rotates orthogonal to the line of the ball
        command.dribbler = 25;
    }
    else {
        command.w = Angle((ballStartPos - interceptPos).angle() + M_PI_2).getAngle();
    }
    publishRobotCommand();
}

//Checks if the ball is kicked to Goal. Kind of duplicate to the condition, but this uses an extra saftey margin
bool InterceptBall::ballToGoal() {
    Vector2 goalCentre = world::field->get_our_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth + GOAL_MARGIN));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth + GOAL_MARGIN);
    LineSegment goal(lowerPost, upperPost);
    Vector2 ballPos = ball->pos;
    Vector2 ballPredPos = Vector2(ball->pos) + Vector2(ball->vel)*Constants::MAX_INTERCEPT_TIME();
    LineSegment ballLine(ballPos, ballPredPos);
    return ballLine.doesIntersect(goal);
}
// Checks if the ball is in our Goal (e.g. the opponent scored)
bool InterceptBall::ballInGoal() {
    Vector2 goalCentre = world::field->get_our_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth);
    Vector2 depth = Vector2(- world::field->get_field().goal_depth, 0.0);
    return control::ControlUtils::pointInRectangle(ball->pos, lowerPost, lowerPost + depth, upperPost + depth,
            upperPost);
}

}//ai
}//rtt