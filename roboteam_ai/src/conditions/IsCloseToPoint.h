//
// Created by robzelluf on 1/23/19.
//

#ifndef ROBOTEAM_AI_ISCLOSETOPOINT_H
#define ROBOTEAM_AI_ISCLOSETOPOINT_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsCloseToPoint : public Condition {
private:
    double margin;
    Vector2 position;
    bool ballPos;
public:
    explicit IsCloseToPoint(std::string name = "IsCloseToPoint", bt::Blackboard::Ptr blackboard = nullptr);
    void initialize() override;
    Status update() override;
    std::string node_name() override;
};

}
}

#endif //ROBOTEAM_AI_ISCLOSETOPOINT_H
