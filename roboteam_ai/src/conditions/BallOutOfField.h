//
// Created by mrlukasbos on 25-1-19.
//

#ifndef ROBOTEAM_AI_BALLOUTOFFIELD_H
#define ROBOTEAM_AI_BALLOUTOFFIELD_H

#include "Condition.h"

namespace rtt {
namespace ai {

class BallOutOfField : public Condition {
public:
    explicit BallOutOfField(std::string name = "BallOutOfField", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    std::string node_name() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_BALLOUTOFFIELD_H
