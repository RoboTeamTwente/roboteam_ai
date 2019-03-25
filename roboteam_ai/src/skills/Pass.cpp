//
// Created by robzelluf on 1/22/19.
//

#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

void Pass::onInitialize() {
    robotToPassToID = coach::g_pass.initiatePass();
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;
    robotToPassTo = World::getRobotForId(static_cast<unsigned int>(robotToPassToID), true);

    bool isBehindBall = coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.30, robotToPassTo->pos, robot->pos);
    bool hasBall = World::ourBotHasBall(robot->id, Constants::MAX_KICK_RANGE());
    bool ballIsMovingFast = Vector2(World::getBall()->vel).length() > 0.4;

    if (ballIsMovingFast) {
        coach::g_pass.setRobotBeingPassedTo(-1);
        coach::g_pass.setPassed(true);
        return Status::Success;
    } else if (isBehindBall) {
        return hasBall ? shoot() : getBall();
    }
    return moveBehindBall();
}

/// this is the method we call when we are far from the desired position
bt::Leaf::Status Pass::moveBehindBall() {
    std::cout << "Getting behind ball" << std::endl;

    targetPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.20, robotToPassTo->pos);
    goToPos.setAvoidBall(true);
    sendMoveCommand(GoToType::NUMERIC_TREES);
    return bt::Leaf::Status::Running;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
bt::Leaf::Status Pass::getBall() {
    std::cout << "Getting ball" << std::endl;

    targetPos = ball->pos;
    goToPos.setAvoidBall(false);
    sendMoveCommand(GoToType::BALL_CONTROL);
    return bt::Leaf::Status::Running;
}

// Now we should have the ball and kick it.
bt::Leaf::Status Pass::shoot() {
    std::cout << "Kicking" << std::endl;

    auto command = getBasicCommand();
    command.kicker = 1;
    command.kicker_forced = 1;
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
    double distance = ((Vector2)ball->pos - robotToPassTo->pos).length();
    double kicker_vel_multiplier = distance > maxPowerDist ? 1.0 : distance / maxPowerDist;

    command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER()*kicker_vel_multiplier);
    publishRobotCommand(command);
    return Status::Running;
}

roboteam_msgs::RobotCommand Pass::getBasicCommand() const {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    return command;
}

/// send a command to move the current robot to targetPos with a certain goToType.
void Pass::sendMoveCommand(const Skill::GoToType& goToType) {
    Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType).vel;
    velocities = control::ControlUtils::VelocityLimiter(velocities);
    roboteam_msgs::RobotCommand command = getBasicCommand();
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = static_cast<float>( (targetPos-robot->pos).angle());
    publishRobotCommand(command);
}

} // ai
} // rtt

