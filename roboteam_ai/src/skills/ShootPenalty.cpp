//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"
#include "../world/World.h"
#include "../world/Ball.h"
#include "../world/Robot.h"

namespace rtt {
namespace ai {

ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

void ShootPenalty::onInitialize() {
    genevaSet = false;
    genevaState = 3;
    tick = 0;
    aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
}

bt::Node::Status ShootPenalty::onUpdate() {
    if (! robot) return Status::Running;

    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.w = robot->angle;
        publishRobotCommand();
        return Status::Running;
    }

    command.w = robot->angle;

    if(!genevaSet) {
        genevaState = determineGenevaState();
        genevaSet = true;
    } else if (tick < genevaChangeTicks) {
        tick++;
    } else {
        auto shotData = robot->getShotController()->getRobotCommand(
                *robot, aimPoint, false, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::HIGH, genevaState);
        command = shotData.makeROSCommand();
    }
    publishRobotCommand();
    return Status::Running;
}

int ShootPenalty::determineGenevaState() {
    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot->pos;
    Vector2 preferredShotVector = aimPoint - ball->pos;

    // determine the angle between the robot position and the shot line
    Angle angleWithShotline = robotToBall.toAngle() - preferredShotVector.toAngle();
    if (angleWithShotline.getAngle() > 0) {
        return genevaState = 2;
    } else {
        return genevaState = 4;
    }
}

}
}