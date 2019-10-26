//
// Created by baris on 27-2-19.
//

#include "skills/PreparePenalty.h"
#include "skills/Skill.h"

namespace rtt {
namespace ai {

PreparePenalty::PreparePenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void PreparePenalty::onInitialize() {
 }


Skill::Status PreparePenalty::onUpdate() {
    return Status::Failure;
}
}
}