//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/NumTreesTracking.h"

namespace rtt::ai::control{

void
NumTreesTracking::trackPath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &currentVelocity,
                                              std::list<rtt::Vector2> &pathPoints, rtt::Vector2 &outputVelocity,
                                              double &outputAngle) {
    int lookAhead = std::min(pathPoints.size()-1, stepsAhead);
    Vector2 currentTarget = *std::next(pathPoints.begin(), lookAhead);
    if (pathPoints.size() > 1 && (currentTarget - currentPosition).length() < minimumDistance){
        pathPoints.erase(pathPoints.begin(), std::next(pathPoints.begin(),lookAhead));
    }

    std::list<Vector2> tempPath(1,currentTarget);
    pidTracking.trackPath(currentPosition, currentVelocity, tempPath, outputVelocity, outputAngle);
}

}