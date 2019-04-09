//
// Created by robzelluf on 4/9/19.
//

#include "Reflect.h"

namespace rtt{
namespace ai{

Reflect::Reflect(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

}
}