//
// Created by mrlukasbos on 1-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H
#define ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H

#include "skills/Receive.h"

namespace rtt::ai {

    class BallPlacementReceive : public Receive {
        public:
        explicit BallPlacementReceive(string name, bt::Blackboard::Ptr blackboard);
        bt::Node::Status onUpdate() override;

        private:
        bool isInPosition(const Vector2 &behindTargetPos) override;
        void moveToCatchPosition(const Vector2 &position);
    };
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H
