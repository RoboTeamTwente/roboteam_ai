//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_DEMOINTERCEPTBALL_H
#define ROBOTEAM_AI_DEMOINTERCEPTBALL_H

#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/BasicPosControl.h>
#include <roboteam_ai/src/skills/InterceptBall.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class DemoInterceptBall : public InterceptBall {
private:
    Arc createKeeperArc();

public:
    explicit DemoInterceptBall(string name, bt::Blackboard::Ptr blackboard);
    Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall) override;

};

}
}

#endif //ROBOTEAM_AI_DEMOINTERCEPTBALL_H
