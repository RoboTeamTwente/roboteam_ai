/*
 * return SUCCESS if the ball is on our side
 * properties:
 * - inField: if true, the ball also has to be in the field to return SUCCESS
 */

#include "conditions/IsBallOnOurSide.h"
#include <world/Ball.h>
#include <world/FieldComputations.h>

namespace rtt::ai {

IsBallOnOurSide::IsBallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

void IsBallOnOurSide::onInitialize() { inField = properties->getBool("inField"); }

bt::Node::Status IsBallOnOurSide::onUpdate() {
    Vector2 ballPos = ball->getPos();

    if (ballPos.x < 0) {
        if (inField) {
            if (FieldComputations::pointIsInField(*field, ballPos)) {
                return Status::Success;
            }
            return Status::Failure;
        }
        return Status::Success;
    }
    return Status::Failure;
}

}  // namespace rtt::ai
