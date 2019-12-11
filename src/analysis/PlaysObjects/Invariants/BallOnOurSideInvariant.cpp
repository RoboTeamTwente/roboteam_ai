//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"
#include <roboteam_utils/Vector2.h>
#include "world/World.h"
#include "world/Field.h"
#include "world/Ball.h"

namespace rtt::ai::analysis {
    bool BallOnOurSide::isTrue(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        Vector2 ballPos = world->getBall()->getPos();

        // force the ball to be in the field
        bool inField = true;

        if (ballPos.x < 0) {

            return abs(ballPos.x) < field->get_field().get(FIELD_LENGTH) / 2 &&
                   abs(ballPos.y) < field->get_field().get(FIELD_WIDTH) / 2;

            return true;
        }
        return false;
    }
}



