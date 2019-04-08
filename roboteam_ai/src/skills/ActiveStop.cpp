//
// Created by baris on 8-4-19.
//

#include "ActiveStop.h"
namespace rtt{
namespace ai {
ActiveStop::ActiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void ActiveStop::onInitialize() {
    goToPos.setStop(true);

}
Skill::Status ActiveStop::onUpdate() {
    return Status::Failure;
}
void ActiveStop::onTerminate(Skill::Status s) {
    Skill::onTerminate(s);
}
}
}