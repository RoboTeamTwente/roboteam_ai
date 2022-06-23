//
// Created by ratoone on 30-01-20.
//

#ifndef RTT_POSITIONCONTROLUTILS_H
#define RTT_POSITIONCONTROLUTILS_H

#include "interface/api/Output.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

class PositionControlUtils {
   public:
    /**
     * If the distance between the old target and the new target > MAX_TARGET_DEVIATION
     * @param targetPos
     * @param oldTarget
     * @return
     */
    static bool isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget);

    /**
     * The target is considered to be reached if the distance between target and currentPosition < MIN_DISTANCE
     * @param targetPos
     * @param currentPosition
     * @return
     */
    static bool isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition);

    /**
     * Is the target moving
     * @param velocity
     */
    static bool isMoving(const Vector2 &velocity);

    static pidVals getPIDValue(const stp::PIDType &pidType) {
        switch (pidType) {
            case stp::PIDType::DEFAULT:
                return interface::Output::getNumTreePid();
            case stp::PIDType::RECEIVE:
                return interface::Output::getReceivePid();
            case stp::PIDType::INTERCEPT:
                return interface::Output::getInterceptPid();
            case stp::PIDType::KEEPER:
                return interface::Output::getKeeperPid();
            case stp::PIDType::KEEPER_INTERCEPT:
                return interface::Output::getKeeperInterceptPid();
        }
    }
};
}  // namespace rtt::ai::control

#endif  // RTT_POSITIONCONTROLUTILS_H
