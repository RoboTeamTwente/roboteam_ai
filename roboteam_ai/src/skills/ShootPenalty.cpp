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
    tick = 0;

    genevaSet = false;
    genevaState = 5;
    double genevaAngle = (robot->getGenevaState() - 3)*10/180.0*M_PI;
    auto theirKeeper = world::world->getRobotClosestToPoint(world::field->get_their_goal_center(),
            WhichRobots::THEIR_ROBOTS);
    if (theirKeeper && theirKeeper->pos.dist(world::field->get_their_goal_center()) < 2.0) {
        std::pair<Vector2, bool> aim = coach::g_offensiveCoach.penaltyAim(ball->pos, robot->angle + genevaAngle,
                theirKeeper->pos);
        aimPoint = aim.first;
        genevaSet = aim.second;
    }
    else {
        aimPoint = world::field->get_their_goal_center();
        genevaSet = false;
    }

}
bt::Node::Status ShootPenalty::onUpdate() {
    if (! robot) return Status::Running;

    if (! genevaSet) {
        genevaState = determineGenevaState();
        genevaSet = true;
    }
    else if (tick < genevaChangeTicks) {
        tick ++;
    }
    else {

        if (fabs(robot->pos.y - ball->pos.y) > 0.03) {
            command = robot->getNumtreePosControl()->getRobotCommand(robot,
                    ball->pos + Vector2(- 0.30, 0)).makeROSCommand();
            command.y_vel *= 2;
            command.x_vel *= 2;
            publishRobotCommand();
            return Status::Running;
        }
        command.w = 0;//(ball->pos - robot->pos).toAngle();
        double vel = abs(sin((double)tick++/20)) * 0.3;
        std::cout << vel << std::endl;
        command.x_vel = vel;
        command.y_vel = 0;

        command.kicker = true;
        command.kicker_vel = Constants::MAX_KICK_POWER();
        command.kicker_forced = robot->hasBall(Constants::MAX_KICK_RANGE());
        command.geneva_state = genevaState;
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
        return genevaState = 1;
    }
    else {
        return genevaState = 5;
    }
}

}
}