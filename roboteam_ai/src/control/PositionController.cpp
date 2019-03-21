//
// Created by thijs on 10-12-18.
//


#include "PositionController.h"

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
        return PositionController::basic(robot, position);
    case PosControlType::NUMERIC_TREES:
        return PositionController::numTree(robot, position);
    case PosControlType::NO_PREFERENCE:
    default:
        return PositionController::numTree(robot, position);
    }
}

PosVelAngle PositionController::ballControl(RobotPtr robot, Vector2 &targetPos) {
    return ballControlController.goToPos(std::move(robot), targetPos);
}

PosVelAngle PositionController::basic(RobotPtr robot, Vector2 &targetPos) {

    Vector2 error = targetPos - robot->pos;

    if (error.length() < rtt::ai::Constants::ROBOT_RADIUS()){
        posPID.setPID(0.5, 0, 0.2);}
    else{
        posPID.setPID(0.5, 0, 0.2);}

    checkInterfacePID();
    PosVelAngle target;
    target.pos = targetPos;
    target.angle = error.angle();
    useforce = true;
    return pidController(robot, target);
}

Vector2 PositionController::force(RobotPtr robot, Vector2 &targetPos) {
    double forceRadius;
    Vector2 error = targetPos - robot->pos;
    if (error.length() < 0.1) {
        if (interface::InterfaceValues::showDebugNumTreeInfo())
            std::cout << "close to target, using basic gtp" << std::endl;
        Vector2 zero = Vector2();
        return zero;
    }
    else if (error.length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*2.0;
    }
    else {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*8.0;
    }

    roboteam_msgs::World world = World::get_world();
    Vector2 force = Vector2();

    for (auto bot : world.us)
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 0.5, forceRadius);
    for (auto bot : world.them)
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 1, forceRadius);
    if (avoidBall)
        force += ControlUtils::calculateForce((Vector2) robot->pos - world.ball.pos, 0.5, forceRadius);
    if (!canGoOutsideField)
        force += Field::pointIsInField(robot->pos, 0.5) ?
                Vector2() : ControlUtils::calculateForce(Vector2(-1.0,-1.0) / robot->pos, 1, 99.9);

    //force = (force.length() > 3.0) ?
    //        force.stretchToLength(3.0) : force;

    return force;
}

PosVelAngle PositionController::numTree(RobotPtr robot, Vector2 &targetPos) {
    PosVelAngle target = numTreeController.goToPos(robot, targetPos);
    if (target.isZero()){
        return basic(robot, targetPos);
    }
    else{
        useforce = false;
        checkInterfacePID();
        }
        return pidController(robot, target);
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

PosVelAngle PositionController::pidController(const RobotPtr &robot, PosVelAngle target) {
    PosVelAngle pidCommand;

    Vector2 pid = Vector2();

    pid = posPID.controlPIR(target.pos-robot->pos, robot->vel);

    pidCommand.pos = target.pos;
    pidCommand.vel += pid.length() < Constants::MAX_VEL() ?
                     pid : pid.stretchToLength(Constants::MAX_VEL());
    pidCommand.angle = (target.pos-robot->pos).angle();


    if (useforce){
        Vector2 temp = pidCommand.vel + force(robot, target.pos);
        pidCommand.vel = temp;
    }

    return pidCommand;
}

/// compare current PID values to those set in the interface
void PositionController::checkInterfacePID() {
    posPID.reset();
    posPID.setPID(interface::InterfaceValues::setNumTreePosP(),
            interface::InterfaceValues::getNumTreePosI(),
            interface::InterfaceValues::getNumTreePosD());
}

} //control
} //ai
} //rtt
