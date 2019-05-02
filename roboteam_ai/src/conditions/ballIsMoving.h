//
// Created by mrlukasbos on 2-5-19.
//

#ifndef ROBOTEAM_AI_BALLISMOVING_H
#define ROBOTEAM_AI_BALLISMOVING_H

#include "Condition.h"

namespace rtt{
namespace ai{

class ballIsMoving : public Condition {
public:
    explicit ballIsMoving(std::string name = "ballIsMoving", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_BALLISMOVING_H
