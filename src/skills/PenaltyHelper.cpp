//
// Created by roboteam on 15-4-19.
//

#include "PenaltyHelper.h"
rtt::ai::PenaltyHelper::PenaltyHelper(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
rtt::ai::Skill::Status rtt::ai::PenaltyHelper::onUpdate() {
    return Status::Failure;
}
void rtt::ai::PenaltyHelper::onInitialize() {
    Skill::onInitialize();
}
void rtt::ai::PenaltyHelper::onTerminate(rtt::ai::Skill::Status s) {
    Skill::onTerminate(s);
}
