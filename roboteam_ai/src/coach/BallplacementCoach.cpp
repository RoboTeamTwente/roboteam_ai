#include "BallplacementCoach.h"

#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/interface/api/Output.h"
#include "roboteam_ai/src/utilities/GameStateManager.hpp"
#include "roboteam_ai/src/interface/api/Input.h"

namespace rtt {
namespace ai {
namespace coach {

// create the global object
BallplacementCoach g_ballPlacement;

/// Return the targetlocation for ballplacement
/// it can be set by the referee or the interface.
Vector2 BallplacementCoach::getBallPlacementPos() {

    // get the ballplacement target from the referee or the interface
    if (interface::Output::usesRefereeCommands()) {
        return GameStateManager::getRefereeData().designated_position;
    }
    return interface::Output::getInterfaceMarkerPosition();
}

/// Get the position where the robot should locate himself before doing ballplacement
/// e.g. it already 'aims' towards to target position.
Vector2 BallplacementCoach::getBallPlacementBeforePos(Vector2 ballPos) {
    Vector2 PlacePos = interface::Output::getInterfaceMarkerPosition();
    Vector2 targetPos = ballPos + (PlacePos - ballPos).stretchToLength(Constants::BP_MOVE_TOWARDS_DIST());
    return targetPos;
}

/// get the position where the robot should locate himself after the ballplacement
/// e.g. it makes sure it does not accidentally hit the ball when driving away.
Vector2 BallplacementCoach::getBallPlacementAfterPos(const std::shared_ptr<world::Robot> &robot) {

    auto ballPos = world::world->getBall()->pos;
    Vector2 target = ballPos + (robot->pos - ballPos).stretchToLength(0.9);

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {ballPos, target},
            Qt::yellow, robot->id, interface::Drawing::LINES_CONNECTED);

    return target;
}

} // coach
} // ai
} // rtt
