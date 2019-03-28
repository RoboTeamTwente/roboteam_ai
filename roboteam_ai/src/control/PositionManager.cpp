/*
 *
 * This class manages cooperation between positioncontrollers
 * Sometimes a position controller cannot find path
 * This class is responsible that in that case another positioncontroller is used
 *
 */

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/positionControllers/ForcePosControl.h>
#include "PositionManager.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace control {

PositionManager::PositionManager() {
    posController = std::make_shared<NumTreePosControl>();
};

PosVelAngle PositionManager::goToPos(RobotPtr robot, Vector2 &position) {
    if (! robot) {
        ROS_ERROR("Error in PositionController->goToPos(robot %i): robot does not exist in world", robot->id);
        return {};
    }
    PosControlType goToType = PosControlType::NUMERIC_TREES;

    //TODO: do stuff that determines which gtp to use...
    return PositionManager::goToPos(std::move(robot), position, goToType);
}

PosVelAngle PositionManager::goToPos(RobotPtr robot, Vector2 &position, PosControlType goToType) {
    if (! robot) return {};

    switch (goToType) {
        case PosControlType::BALL_CONTROL:
            posController = std::make_shared<ControlGoToPosBallControl>();
            break;
        case PosControlType::BASIC:
            posController = std::make_shared<BasicPosControl>();
            break;
        case PosControlType::FORCE:
            posController = std::make_shared<ForcePosControl>();
            break;
        case PosControlType::NUMERIC_TREES:
            return PositionManager::numTree(robot, position);
        default:
            return PositionManager::numTree(robot, position);
    }
}


//PosVelAngle PositionManager::numTree(RobotPtr robot, Vector2 &targetPos) {
//    PosVelAngle target = numTreeController.goToPos(robot, targetPos);
//    if (target.isZero()) {
//        return force(robot, targetPos);
//    }
//    else {
//        return pidController(robot, target);
//    }
//}


} //control
} //ai
} //rtt
