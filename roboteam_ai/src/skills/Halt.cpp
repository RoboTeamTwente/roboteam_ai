//
// Created by mrlukasbos on 6-12-18.
//

#include "Halt.h"

namespace rtt {
namespace ai {

Halt::Halt(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Halt::onInitialize() {
    if (properties->hasInt("ticks")) {
        ticks = properties->getInt("ticks");
    }
    tick = 0;
}

Halt::Status Halt::onUpdate() {
    if (tick <= ticks) {
        std::cout<<"COMMAND"<<std::endl;
        // send slowing down command
        command.use_angle = 0;
        publishRobotCommand();
        tick ++;
        return Status::Running;
    }
    else{
        // do not send a command
        std::cout<<"NO COMMAND"<<std::endl;
        return Status::Running;
    }
}

} // ai
} // rtt