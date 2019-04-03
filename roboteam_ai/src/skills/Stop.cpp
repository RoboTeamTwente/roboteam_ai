//
// Created by baris on 3-4-19.
//

#include "Stop.h"
namespace rtt {
namespace ai {

Stop::Stop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void Stop::onInitialize() {
    Skill::onInitialize();
}
Skill::Status Stop::onUpdate() {
    return Status::Success;
}
void Stop::onTerminate(Skill::Status s) {
    Skill::onTerminate(s);
}
}
}