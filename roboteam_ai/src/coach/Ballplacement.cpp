//
// Created by mrlukasbos on 19-3-19.
//

/*
 *
 * This class gives handles for all positioning needed for ballplacement.
 *
 */

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "Ballplacement.h"

namespace rtt {
namespace ai {
namespace coach {

// create the global object
Ballplacement g_ballPlacement;

/// Return the targetlocation for ballplacement
/// it can be set by the referee or the interface.
Vector2 Ballplacement::getBallPlacementPos(){

    // TODO use the referee for this if we use referee
    return interface::InterfaceValues::getBallPlacementTarget();
}

/// Get the position where the robot should locate himself before doing ballplacement
/// e.g. it already 'aims' towards to target position.
Vector2 Ballplacement::getBallPlacementBeforePos(Vector2 ballPos){
    Vector2 PlacePos=interface::InterfaceValues::getBallPlacementTarget();
    Vector2 targetPos=ballPos + (PlacePos - ballPos).stretchToLength(Constants::BP_MOVE_TOWARDS_DIST());
    return targetPos;
}

/// get the position where the robot should locate himself after the ballplacement
/// e.g. it makes sure it does not accidentally hit the ball when driving away.
Vector2 Ballplacement::getBallPlacementAfterPos(double RobotAngle){
    Vector2 targetPos=interface::InterfaceValues::getBallPlacementTarget() + Vector2(Constants::BP_MOVE_BACK_DIST(),0).rotate(RobotAngle+M_PI);
    return targetPos;
}

} // coach
} // ai
} // rtt
