//
// Created by robzelluf on 4/9/19.
//

#include "ReflectKick.h"

namespace rtt {
namespace ai {

ReflectKick::ReflectKick(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void ReflectKick::onInitialize() { }

ReflectKick::Status ReflectKick::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) return Status::Failure;

    return Status::Running;
}

void ReflectKick::onTerminate(Status s) { }

Vector2 ReflectKick::getFarSideOfGoal() {
    Vector2 robotPos = robot->pos;
    float cornering = rtt::ai::world::field->get_field().goal_width/2.0;
    if (robotPos.y >= 0) {
        return {rtt::ai::world::field->get_their_goal_center().x,
                rtt::ai::world::field->get_their_goal_center().y + cornering};
    }
    else {
        return {rtt::ai::world::field->get_their_goal_center().x,
                rtt::ai::world::field->get_their_goal_center().y - cornering};
    }
}

}
}