//
// Created by mrlukasbos on 19-3-19.
//

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "Ballplacement.h"

namespace rtt {
namespace ai {
namespace coach {

Ballplacement g_ballPlacement;

Vector2 Ballplacement::getBallPlacementPos(){

    // TODO use the referee for this if we use referee
    return interface::InterfaceValues::getBallPlacementTarget();
}

Vector2 Ballplacement::getBallPlacementBeforePos(Vector2 ballPos){
    Vector2 PlacePos=interface::InterfaceValues::getBallPlacementTarget();
    Vector2 targetPos=ballPos + (PlacePos - ballPos).stretchToLength(Constants::BP_MOVE_TOWARDS_DIST());
    return targetPos;
}

Vector2 Ballplacement::getBallPlacementAfterPos(double RobotAngle){
    Vector2 targetPos=interface::InterfaceValues::getBallPlacementTarget() + Vector2(Constants::BP_MOVE_BACK_DIST(),0).rotate(RobotAngle+M_PI);
    return targetPos;
}

} // coach
} // ai
} // rtt
