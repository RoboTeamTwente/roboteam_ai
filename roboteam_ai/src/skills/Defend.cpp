#include "Defend.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

Defend::Defend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Defend::onInitialize() {

}


bt::Node::Status Defend::onUpdate() {

    coach::Coach =
    return bt::Node::Status::Running;
}


void Defend::onTerminate(bt::Node::Status s) {

}

} // ai
} // rtt