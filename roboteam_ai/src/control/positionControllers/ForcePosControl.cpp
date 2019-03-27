//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "ForcePosControl.h"

namespace rtt {
namespace ai {
namespace control {

PosVelAngle ForcePosControl::getPosVelAngle(RobotPtr robot, Vector2 &targetPos) {
    double forceRadius;

    if ((targetPos - robot->pos).length() < 0.1) {
        if (interface::InterfaceValues::showDebugNumTreeInfo()) {
            std::cout << "close to target, using basic gtp" << std::endl;
        }
        return basic(robot, targetPos);

    } else if ((targetPos - robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 2.0;
        initializePID(3.0, 1.0, 0.2);
    } else {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 8.0;
        initializePID(3.0, 0.5, 1.5);
    }

    PosVelAngle target;
    roboteam_msgs::World world = World::get_world();
    Vector2 force = (targetPos - robot->pos);
    force = (force.length() > 3.0) ?
            force.stretchToLength(3.0) : force;

    for (auto bot : world.us) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 1, forceRadius);
    }

    for (auto bot : world.them) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 2, forceRadius);
    }

    if (avoidBall) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - world.ball.pos, 1, forceRadius);
    }

    if (!canMoveOutOfField) {
        force += Field::pointIsInField(robot->pos, 0.5) ?
                 Vector2() : ControlUtils::calculateForce(Vector2(-1.0, -1.0) / robot->pos, 1, 99.9);
    }

    force = (force.length() > 3.0) ? force.stretchToLength(3.0) : force;

    target.vel = force;
    return PIDController(robot, target);
}

}
}
}
}
