//
// Created by baris on 8-4-19.
//

#include <roboteam_ai/src/coach/FormationCoach.h>
#include "PassiveStop.h"
namespace rtt{
namespace ai {

PassiveStop::PassiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void PassiveStop::onInitialize() {
    goToPos.setStop(true);
    coach::g_formation.registerPassive(robot->id);
}
Skill::Status PassiveStop::onUpdate() {
    return Status::Failure;
}
void PassiveStop::onTerminate(Skill::Status s) {
    Skill::onTerminate(s);
}
Vector2 PassiveStop::getTargetPos() {
    return Vector2();
}
}
}