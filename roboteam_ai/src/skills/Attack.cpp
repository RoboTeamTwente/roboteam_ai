//
// Created by thijs on 17-12-18.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include "Attack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Attack::onInitialize() {
    gtp = std::make_shared<control::NumTreePosControl>();
    gtp->setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());
    shot = false;
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && !world::world->robotHasBall(robot->id, world::OUR_ROBOTS)) {
        return Status::Success;
    }

    Vector2 ball = world::world->getBall()->pos;
    control::ShotData shotData = shotControl.getShotData(* robot, world::field->get_their_goal_center());


    command.x_vel = shotData.vel.x;
    command.y_vel = shotData.vel.y;
    command.w = shotData.angle.getAngle();
    command.kicker = shotData.kick;
    command.kicker_forced = shotData.kick;
    command.kicker_vel = shotData.kickSpeed;
    command.geneva_state = shotData.genevaState;


//    Vector2 velocity;
//    if (world::field->pointIsInDefenceArea(robot->pos, false, 0.0)) {
//        velocity = ((Vector2) robot->pos - world::field->get_our_goal_center()).stretchToLength(2.0);
//    }
//    else if (world::field->pointIsInDefenceArea(robot->pos, false, 0.0)) {
//        velocity = ((Vector2) robot->pos - world::field->get_their_goal_center()).stretchToLength(2.0);
//    }
//    else if (world::field->pointIsInDefenceArea(ball, false) || world::field->pointIsInDefenceArea(ball, true)) {
//        velocity = {0, 0};
//    }
//    else if (world::field->pointIsInDefenceArea(targetPos, false)) {
//        velocity = {0, 0};
//    }
//    else {
//        velocity = gtp->getPosVelAngle(robot, targetPos).vel;
//    }
//
//    velocity = control::ControlUtils::velocityLimiter(velocity);
//
//    command.x_vel = static_cast<float>(velocity.x);
//    command.y_vel = static_cast<float>(velocity.y);

    publishRobotCommand();

    return Status::Running;
}

void Attack::onTerminate(Status s) {

    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand();
}

} // ai
} // rtt