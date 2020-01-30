//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/NumTreesTracking.h"

namespace rtt::ai::control{

Position NumTreesTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                              std::vector<Vector2> &pathPoints) {
    int lookAhead = std::min(pathPoints.size()-1, STEPS_AHEAD);
    Vector2 currentTarget = *std::next(pathPoints.begin(), lookAhead);
    if (pathPoints.size() > 1 && PositionControlUtils::isTargetReached(currentTarget, currentPosition)){
        //track the Nth point, or the last if the size is smaller than N; the untracked ones are discarded
        pathPoints.erase(pathPoints.begin(), std::next(pathPoints.begin(),lookAhead));
    }

    std::vector<Vector2> tempPath{currentTarget};
    return pidTracking.trackPath(currentPosition, currentVelocity, tempPath);
}

}