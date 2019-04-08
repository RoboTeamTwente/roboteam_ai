//
// Created by mrlukasbos on 25-1-19.
//

#ifndef ROBOTEAM_AI_BALLOUTOFFIELD_H
#define ROBOTEAM_AI_BALLOUTOFFIELD_H

#include "Condition.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {

class CanShootToTarget : public Condition {
public:
    explicit CanShootToTarget(std::string name = "CanShootToTarget", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    std::string node_name() override;

private:
	Vector2 target;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_BALLOUTOFFIELD_H
