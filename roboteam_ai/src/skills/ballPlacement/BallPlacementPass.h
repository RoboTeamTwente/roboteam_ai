//
// Created by mrlukasbos on 3-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTPASS_H
#define ROBOTEAM_AI_BALLPLACEMENTPASS_H

#include "roboteam_ai/src/skills/Pass.h"

namespace rtt {
namespace ai {

class BallPlacementPass : public Pass {

public:
    explicit BallPlacementPass(string name, bt::Blackboard::Ptr blackboard);
    bt::Node::Status onUpdate() override;
    void onInitialize() override;

};

}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTPASS_H
