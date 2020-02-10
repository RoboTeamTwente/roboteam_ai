//
// Created by rolf on 11-4-19.
//

#include "skills/SlingShot.h"

#include "control/ControlUtils.h"
#include "world/Ball.h"
#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai {

SlingShot::SlingShot(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}
void SlingShot::onInitialize() {
    waitingTicks = 0;
    dribbledTicks = 0;
    ballShotTicks = 0;
    if (!world->robotHasBall(robot->id, true)) {
        progression = FAIL;
        return;
    }
    progression = DRIBBLING;
    rotateAngle = robot->angle;
    kickOrient = robot->angle;
}
void SlingShot::onTerminate(rtt::ai::Skill::Status s) {
    waitingTicks = 0;
    dribbledTicks = 0;
    ballShotTicks = 0;
    if (!world->robotHasBall(robot->id, true)) {
        progression = FAIL;
        return;
    }
    progression = DRIBBLING;
    rotateAngle = robot->angle;
    kickOrient = robot->angle;
}
Skill::Status SlingShot::onUpdate() {
    progression = updateProgress(progression);
    switch (progression) {
        case DRIBBLING: {
            sendDribbleCommand();
            return Status::Running;
        }
        case ROTATINGAWAY: {
            sendRotateCommand();
            return Status::Running;
        }
        case WAITINGFORRESULT: {
            sendWaitCommand();
            return Status::Running;
        }
        case FAIL: {
            sendWaitCommand();
            return Status::Failure;
        }
        case SUCCESS: {
            sendWaitCommand();
            return Status::Success;
        }
    }
    return Status::Failure;
}
SlingShot::Progression SlingShot::updateProgress(Progression currentProgress) {
    // we go from dribbling to rotating away to waiting for a result.
    if (currentProgress == DRIBBLING) {
        if (dribbledTicks < maxDribbleTicks) {
            dribbledTicks++;
            return world->robotHasBall(robot->id, true) ? DRIBBLING : FAIL;
        } else {
            setRotate();
            return ROTATINGAWAY;
        }
    }  // if the robot is at the angle we start waiting
    else if (currentProgress == ROTATINGAWAY) {
        return robotAtAngle() ? WAITINGFORRESULT : ROTATINGAWAY;
    } else if (currentProgress == WAITINGFORRESULT) {
        waitingTicks++;
        if (ballShot()) {
            ballShotTicks++;
            if (ballShotTicks > 3) {
                return SUCCESS;
            }
        }
        return waitingTicks > maxWaitingTicks ? FAIL : WAITINGFORRESULT;  // just continue or stop if it takes too long
    }
    return FAIL;
}
bool SlingShot::ballShot() {
    Vector2 vectorFromStart = ball->getPos() - kickPos;
    double vectorFromStartAngle = vectorFromStart.angle();
    double angleDif = control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(kickOrient), control::ControlUtils::constrainAngle(vectorFromStartAngle));
    // check if the ball has gone to direction we expect for more than 2 centimeters
    return angleDif > M_PI_2 && vectorFromStart.length() > 0.02;
}
bool SlingShot::robotAtAngle() {
    double margin = 0.03 * M_PI;
    return control::ControlUtils::angleDifference(robot->angle.getAngle(), control::ControlUtils::constrainAngle(rotateAngle)) < margin;
}
void SlingShot::sendDribbleCommand() {
    command.set_dribbler(31);  // TODO:check if we can control velocities
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    command.set_w(robot->angle);
    publishRobotCommand();
}
void SlingShot::sendRotateCommand() {
    Vector2 position = kickPos + Vector2(0.2, 0).rotate(rotateAngle + M_PI);
    auto velocities = gtp.getRobotCommand(world, field, robot, position).vel;
    velocities = control::ControlUtils::velocityLimiter(velocities, 1.5);
    command.set_dribbler(0);
    command.mutable_vel()->set_x(velocities.x);
    command.mutable_vel()->set_y(velocities.y);
    command.set_w(rotateAngle);
    publishRobotCommand();
}
void SlingShot::sendWaitCommand() {
    command.set_dribbler(0);
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    command.set_w(rotateAngle);
    publishRobotCommand();
}
void SlingShot::setRotate() {
    kickOrient = robot->angle;
    kickPos = robot->pos;
    rotateAngle = control::ControlUtils::constrainAngle(robot->angle + M_PI_2);
}
}  // namespace rtt::ai