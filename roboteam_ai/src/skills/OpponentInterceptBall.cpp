//
// Created by rolf on 12/12/18.
//

#include "OpponentInterceptBall.h"
#include "roboteam_ai/src/interface/api/Input.h"
#include "roboteam_ai/src/world/Field.h"

namespace rtt {
namespace ai {

OpponentInterceptBall::OpponentInterceptBall(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { };

//TODO: make prediction for the RobotPtr if it can even intercept the ball at all from the initialization state.
void OpponentInterceptBall::onInitialize() {
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

OpponentInterceptBall::Status OpponentInterceptBall::onUpdate() {
    ball = world::world->getBall();
    //The keeper dynamically updates the intercept position as he needs to be responsive and cover the whole goal and this would help against curveballs etc.
    if (keeper) {
        interceptPos = computeInterceptPoint(ball->pos,
                                             Vector2(ball->pos) + Vector2(ball->vel)*Constants::MAX_INTERCEPT_TIME());
    }
    deltaPos = interceptPos - robot->pos;
    checkProgression();

    interface::Input::drawData(interface::Visual::INTERCEPT, {ballStartPos, ballEndPos}, Qt::darkCyan, robot->id,
                               interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::INTERCEPT, {interceptPos}, Qt::cyan, robot->id,
                               interface::Drawing::DOTS, 5, 5);
    // if we are not already rotating
    if (currentProgression != INPOSITION && deltaPos.length() > TURNING_DISTANCE) {
        // update if we want to rotate or not; if we have time to turn we do so, otherwise not.
        // this assumes we are at 90 to 180 degrees difference with the target angle; can be improved by measuring average turn time under keeper circumstances
        stayAtOrientation = ball->vel.length()*TURN_TIME > (interceptPos - ball->pos).length();
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

void OpponentInterceptBall::sendMoveCommand(Vector2 targetPos) {
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
    if (deltaPos.length() > 0.05) {
        auto blockAngle = Angle((interceptPos - robot->pos).angle());
        command.w = ! backwards ? blockAngle.getAngle() : Angle(blockAngle + M_PI).getAngle();
    }
    else {
        if (! stayAtOrientation) {
            command.w = (ballStartPos - interceptPos).angle();
        }
        else {
            command.w = Angle((ballStartPos - interceptPos).angle() + M_PI_2);
        }
    }
    publishRobotCommand();
}

void OpponentInterceptBall::checkProgression() {
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

}

void OpponentInterceptBall::onTerminate(rtt::ai::Skill::Status s) {
    sendStopCommand();
    tickCount = 0;
    currentProgression = INTERCEPTING;
}

Vector2 OpponentInterceptBall::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;
    if (keeper) {
        // Depends on two keeper Constants in Constants!
        Arc keeperCircle = createKeeperArc();
        std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = keeperCircle.intersectionWithLine(
                startBall, endBall);
        if (intersections.first && intersections.second) {
            double dist1 = (Vector2(robot->pos) - *intersections.first).length();
            double dist2 = (Vector2(robot->pos) - *intersections.second).length();
            if (dist2 < dist1) {
                interceptionPoint = *intersections.second;
            }
            else {
                interceptionPoint = *intersections.first;
            }
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
            Line shotLine(startBall, endBall);
            interceptionPoint = shotLine.project(robot->pos);
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
bool OpponentInterceptBall::missedBall(Vector2 startBall, Vector2 endBall, Vector2 ballVel) {
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
bool OpponentInterceptBall::ballDeflected() {
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
void OpponentInterceptBall::sendStopCommand() {
    command.x_vel = 0;
    command.y_vel = 0;
    if (! stayAtOrientation) {
        command.w = static_cast<float>((ballStartPos
                                        - interceptPos).angle()); //Rotates orthogonal to the line of the ball
    }
    else {
        command.w = Angle((ballStartPos - interceptPos).angle() + M_PI_2).getAngle();
    }
    publishRobotCommand();
}

//Checks if the ball is kicked to Goal. Kind of duplicate to the condition, but this uses an extra saftey margin
bool OpponentInterceptBall::ballToGoal() {
    Vector2 goalCentre = world::field->get_their_goal_center();
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
bool OpponentInterceptBall::ballInGoal() {
    Vector2 goalCentre = world::field->get_their_goal_center();
    double goalWidth = world::field->get_field().goal_width;
    Vector2 lowerPost = goalCentre + Vector2(0.0, - (goalWidth));
    Vector2 upperPost = goalCentre + Vector2(0.0, goalWidth);
    Vector2 depth = Vector2(- world::field->get_field().goal_depth, 0.0);
    return control::ControlUtils::pointInRectangle(ball->pos, lowerPost, lowerPost + depth, upperPost + depth,
                                                   upperPost);
}

Arc OpponentInterceptBall::createKeeperArc() {
    double goalwidth = rtt::ai::world::field->get_field().goal_width;
    Vector2 goalPos = rtt::ai::world::field->get_their_goal_center();
    double diff = rtt::ai::Constants::KEEPER_POST_MARGIN() - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN();

    double radius = diff*0.5 + goalwidth*goalwidth/(8*diff); //Pythagoras' theorem.
    double angle = asin(goalwidth/2/radius); // maximum angle (at which we hit the posts)
    Vector2 center = Vector2(goalPos.x - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN() - radius, 0);

    return diff > 0 ? rtt::Arc(center, radius, - M_PI + angle,  - angle + M_PI) :
           rtt::Arc(center, radius, - angle, angle);
}

}//ai
}//rtt