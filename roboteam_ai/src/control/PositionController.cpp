//
// Created by thijs on 10-12-18.
//


#include "PositionController.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace control {

PositionController::PositionController() = default;

void PositionController::clear(PosControlType goToType) {

    switch (goToType) {
    case PosControlType::NO_PREFERENCE:break;
    case PosControlType::BALL_CONTROL:break;
    case PosControlType::BASIC:break;
    case PosControlType::FORCE:break;
    case PosControlType::NUMERIC_TREES: {
        numTreeController.clear();
        break;
    }
    default:break;
    }
}

PosVelAngle PositionController::goToPos(RobotPtr robot, Vector2 &position) {
    return PositionController::goToPos(robot, position, NO_PREFERENCE);
}

PosVelAngle PositionController::goToPos(RobotPtr robot, Vector2 &position, PosControlType goToType) {
    if (! robot) {
        ROS_ERROR("Error in PositionController->goToPos(robot %i): robot does not exist in world", robot->id);
        return {};
    }

    posPID.reset();
    switch (goToType) {
    case PosControlType::BALL_CONTROL:
        return PositionController::ballControl(robot, position);
    case PosControlType::BASIC:
        return PositionController::basic(robot, position);
    case PosControlType::FORCE:
        return PositionController::force(robot, position);
    case PosControlType::NUMERIC_TREES:
        return PositionController::numTree(robot, position);
    case PosControlType::NO_PREFERENCE:
        return PositionController::numTree(robot, position);
    default:
        return PositionController::numTree(robot, position);
    }
}

PosVelAngle PositionController::ballControl(RobotPtr robot, Vector2 &targetPos) {
    return ballControlController.goToPos(std::move(robot), targetPos);
}

PosVelAngle PositionController::basic(RobotPtr robot, Vector2 &targetPos) {
    //Just a PID position controller
    Vector2 error = targetPos - robot->pos;

    posPID.setPID(15.0, 0, 10.0);//increase P for more speed, increase D for less overshoot

    PosVelAngle target;
    target.pos.x = robot->pos.x + error.stretchToLength(1).x;//Makes the target max 1 meter to the front of the robot
    target.pos.y = robot->pos.y + error.stretchToLength(1).y;
    target.angle = error.angle();
    userotforce = false;
    return pidController(robot, target, 0.5);
}

PosVelAngle PositionController::force(RobotPtr robot, Vector2 &targetPos) {
    //Just a PID position controller but also rotates the velocity afterwards to avoid objects
    Vector2 error = targetPos - robot->pos;

    posPID.setPID(15.0, 0, 10.0);//increase P for more speed, increase D for less overshoot

    checkInterfacePID();
    PosVelAngle target;
    target.pos.x = robot->pos.x + error.stretchToLength(1).x;//Makes the target max 1 meter to the front of the robot
    target.pos.y = robot->pos.y + error.stretchToLength(1).y;
    target.angle = error.angle();
    userotforce = true;
    return pidController(robot, target, 0.5);
}


double PositionController::rotation(RobotPtr robot, Vector2 &targetPos, Vector2 &targetVel, float weight) {
    double forceRadius;
    Vector2 error = targetPos - robot->pos;
    if (error.length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*2.0;
    }
    else {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*8.0;
    }

    roboteam_msgs::World world = World::get_world();
    double force = 0.0;

    for (auto bot : world.us){
        force += ControlUtils::calculateRotForce((Vector2) bot.pos - robot->pos, targetVel, weight/2, forceRadius);}
    for (auto bot : world.them){
        force += ControlUtils::calculateRotForce((Vector2) bot.pos - robot->pos, targetVel, weight, forceRadius);}
    if (avoidBall){
        force += ControlUtils::calculateRotForce((Vector2) world.ball.pos - robot->pos, targetVel, weight, forceRadius);}

    force = (force > 1) ?
            1 : force;
    force = (force < -1) ?
            -1 : force;

    return force;
}


PosVelAngle PositionController::numTree(RobotPtr robot, Vector2 &targetPos) {
    PosVelAngle target = numTreeController.goToPos(robot, targetPos);
    if (target.isZero()) {
        return force(robot, targetPos);
    }
    else {
        posPID.setPID(20.0, 0, 10.0);//increase P for more speed, increase D for less overshoot
        userotforce = true;
        checkInterfacePID();//For Testing, otherwise I(Koen) found the above values to sorta work.
    }
    return pidController(robot, target, 0.10);
}

void PositionController::setAvoidBall(bool _avoidBall) {
    // Add a function to avoid the ball for all goToPos's
    numTreeController.setAvoidBall(_avoidBall);
    avoidBall = _avoidBall;
}

void PositionController::setCanGoOutsideField(bool _canGoOutsideField) {
    // Add a function to make sure the robot does not go out of the field for all goToPos's
    numTreeController.setCanGoOutsideField(_canGoOutsideField);
    canGoOutsideField = _canGoOutsideField;
}

PosVelAngle PositionController::pidController(const RobotPtr &robot, PosVelAngle target, float weight) {
    PosVelAngle pidCommand;

    Vector2 pid = Vector2();

    pid = posPID.controlPIR(target.pos-robot->pos, robot->vel);

    pidCommand.pos = target.pos;
    pidCommand.vel = pid.length() < Constants::MAX_VEL() ?
                     pid : pid.stretchToLength(Constants::MAX_VEL());
    pidCommand.angle = (target.pos-robot->pos).angle();


    if (userotforce){
        double rot = rotation(robot, pidCommand.pos, pidCommand.vel, weight);
        pidCommand.vel = pidCommand.vel.rotate(rot*0.5*M_PI);
    }

    return pidCommand;
}

/// compare current PID values to those set in the interface
void PositionController::checkInterfacePID() {
    posPID.reset(); //Throws away the integrated I but we don't care since it does not work for this situation.
    posPID.setPID(interface::InterfaceValues::setNumTreePosP(),
            interface::InterfaceValues::getNumTreePosI(),
            interface::InterfaceValues::getNumTreePosD());
}

} //control
} //ai
} //rtt
