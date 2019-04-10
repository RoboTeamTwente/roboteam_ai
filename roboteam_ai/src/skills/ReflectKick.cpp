//
// Created by robzelluf on 4/9/19.
//

#include "ReflectKick.h"

namespace rtt{
namespace ai{

ReflectKick::ReflectKick(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

void ReflectKick::onInitialize() {}

ReflectKick::Status ReflectKick::onUpdate() {
    if(coach::g_pass.getRobotBeingPassedTo() != robot->id) return Status::Failure;


    return Status::Running;
}

void ReflectKick::onTerminate(Status s) {}

}
}