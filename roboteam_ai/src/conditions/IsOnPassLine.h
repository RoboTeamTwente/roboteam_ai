//
// Created by robzelluf on 4/25/19.
//

#include "Condition.h"
#include <roboteam_ai/src/coach/PassCoach.h>

#ifndef ROBOTEAM_AI_ISPASSHAPPENING_H
#define ROBOTEAM_AI_ISPASSHAPPENING_H

namespace rtt {
namespace ai {

class IsOnPassLine : public Condition {
private:
    const double DISTANCE_FROM_PASS_LINE = 3 * Constants::ROBOT_RADIUS();
public:
    explicit IsOnPassLine(std::string name = "IsOnPassLine", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

}
}



#endif //ROBOTEAM_AI_ISPASSHAPPENING_H
