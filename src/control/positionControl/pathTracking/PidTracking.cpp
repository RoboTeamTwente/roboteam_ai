//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/PidTracking.h"

namespace rtt::ai::control {

Position PidTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId, double angle, stp::PIDType pidType) {
    PositionControlUtils::removeFirstIfReached(pathPoints, currentPosition);
    if (pidMapping.find(robotId) == pidMapping.end()) {
        pidMapping[robotId] = std::make_pair(PID(), PID());
        pidMapping[robotId].first.setMaxIOutput(MAX_VELOCITY);
        pidMapping[robotId].second.setMaxIOutput(MAX_VELOCITY);
    }

    updatePIDValues(pidType, robotId);

    Vector2 velocity{};
    velocity.x = pidMapping[robotId].first.getOutput(currentPosition.x, pathPoints.front().x);
    velocity.y = pidMapping[robotId].second.getOutput(currentPosition.y, pathPoints.front().y);

    return {velocity, angle};
}

Position PidTracking::trackVelocity(const Vector2 &currentVelocity, std::vector<Vector2> &velocityPoints, int robotId, stp::PIDType pidType) {
    bool newBool = true;
    //TODO: fine tune MIN_DISTANCE_VELOCITY_REACHED
    double MIN_DISTANCE_VELOCITY_REACHED = 0.01;
    while(newBool && velocityPoints.size() > 1) { // while loop that erases all velocities that have already been passed
        // if true, you want to accelerate and velocityPoints.front() is already lower than current velocity
        if(velocityPoints.at(2).length() > velocityPoints.at(1).length() &&
            currentVelocity.length() > velocityPoints.at(1).length()) {
            velocityPoints.erase(velocityPoints.begin());
        }
        // if true, you want to decelerate and velocityPoints.front() is already higher than current velocity
        else if(velocityPoints.at(2).length() < velocityPoints.at(1).length() &&
                 currentVelocity.length() < velocityPoints.at(1).length()) {
            velocityPoints.erase(velocityPoints.begin());
        }
        // if true, velocity remains the same so only remove velocityPoints.front() and stop the while loop
        else if (velocityPoints.at(2).length() == velocityPoints.at(1).length() &&
                 abs(currentVelocity.length() - velocityPoints.at(1).length()) < MIN_DISTANCE_VELOCITY_REACHED) {
            velocityPoints.erase(velocityPoints.begin());
            newBool = false;
        }
        // else, velocityPoint.front() has not been reached yet so don't remove a velocityPoint and stop the while loop
        else { newBool = false; }
    }
    if (pidMapping.find(robotId) == pidMapping.end()) {
        pidMapping[robotId] = std::make_pair(PID(), PID());
        pidMapping[robotId].first.setMaxIOutput(MAX_VELOCITY);
        pidMapping[robotId].second.setMaxIOutput(MAX_VELOCITY);
    }

    updatePIDValues(pidType, robotId);

    Vector2 velocity{};
    velocity.x = pidMapping[robotId].first.getOutput(currentVelocity.x, velocityPoints.front().x);
    velocity.y = pidMapping[robotId].second.getOutput(currentVelocity.y, velocityPoints.front().y);

    return {velocity, velocity.angle()};
}

void PidTracking::updatePidValuesFromInterface(bool isKeeper) {
    auto newPid = isKeeper ? interface::Output::getKeeperPid() : interface::Output::getNumTreePid();
    for (auto pid : pidMapping) {
        pidMapping[pid.first].first.setPID(newPid);
        pidMapping[pid.first].second.setPID(newPid);
    }
}

void PidTracking::updatePIDValues(stp::PIDType pidType, int robotID) {
    std::tuple<double, double, double> newPID;

    switch (pidType) {
      case stp::PIDType::DEFAULT: {
            newPID = interface::Output::getNumTreePid();
            break;
        }
        case stp::PIDType::RECEIVE: {
            newPID = interface::Output::getReceivePid();
            break;
        }
        case stp::PIDType::INTERCEPT: {
            newPID = interface::Output::getInterceptPid();
        }
        case stp::PIDType::KEEPER: {
            newPID = interface::Output::getKeeperPid();
            break;
        }
        case stp::PIDType::KEEPER_INTERCEPT: {
            newPID = interface::Output::getKeeperInterceptPid();
            break;
        }
        case stp::PIDType::BBT: {
            //TODO: fine tune PID variables
            newPID = pidVals(6.0, 0.0, 1.0);
            break;
        }
    }

    pidMapping[robotID].first.setPID(newPID);
    pidMapping[robotID].second.setPID(newPID);
}
}  // namespace rtt::ai::control