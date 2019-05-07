//
// Created by mrlukasbos on 1-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H
#define ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H

#include "roboteam_ai/src/skills/Receive.h"

namespace rtt {
namespace ai {

class BallPlacementReceive : public Receive {

public:
    explicit BallPlacementReceive(string name, bt::Blackboard::Ptr blackboard);
    bt::Node::Status onUpdate() override;
private:
    bool isInPosition(const Vector2 &behindTargetPos) override;
    void moveToCatchPosition(Vector2 position);

};
}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTRECEIVE_H
