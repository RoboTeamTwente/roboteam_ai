//
// Created by mrlukasbos on 3-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTPASS_H
#define ROBOTEAM_AI_BALLPLACEMENTPASS_H

#include "skills/Pass.h"

namespace rtt::ai {

class BallPlacementPass : public Pass {
   public:
    explicit BallPlacementPass(std::string name, bt::Blackboard::Ptr blackboard);
    bt::Node::Status onUpdate() override;
    void onInitialize() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPLACEMENTPASS_H
