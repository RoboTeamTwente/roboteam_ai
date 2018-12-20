//
// Created by rolf on 19-10-18.
//

#include "HasBall.hpp"

namespace rtt {
namespace ai {

HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status HasBall::update() {
    robot = getRobotFromProperties(properties);

    if (! robot) return Status::Failure;

    auto ball = World::getBall();
    if (botHasBall(ball.pos)) return Status::Success;
    else return Status::Failure;

}

bool HasBall::botHasBall(Vector2 ballPos) {

    Vector2 deltaPos = (ballPos - robot->pos);

    double dist = deltaPos.length();
    double angle = deltaPos.angle();

    //TODO: TEST if this is from centre of dribbler of robot in practice. What does
    // Within 15 cm and .4 radians (of center of dribbler)
    return ((dist < 0.15) && (fabs(angle - robot->angle) < 0.4));
}

} // ai
} // rtt