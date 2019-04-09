//
// Created by mrlukasbos on 25-1-19.
//

#ifndef ROBOTEAM_AI_CANSHOOTTOTARGET_H
#define ROBOTEAM_AI_CANSHOOTTOTARGET_H

#include "Condition.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {

class CanShootToTarget : public Condition {
    const double MAX_DIST_FROM_LINE = Constants::ROBOT_RADIUS();
public:
    explicit CanShootToTarget(std::string name = "CanShootToTarget", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    std::string node_name() override;

private:
	Vector2 target;
	double margin;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_BALLOUTOFFIELD_H
