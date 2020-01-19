//
// Created by rolf on 18-01-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2DAsync.h"
namespace rtt {
template<class num>
BBTrajectory2DAsync<num>::BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc, const Vector2 &lineDirection) {
    generateAsyncTrajectory(initialPos,initialVel,finalPos,maxVel,maxAcc,lineDirection);
}
template<class num>
void BBTrajectory2DAsync<num>::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc,num alpha) noexcept {
    x = BBTrajectory1D<num>(initialPos.x, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y = BBTrajectory1D<num>(initialPos.y, initialVel.y, finalPos.y, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}
template<class num>
void BBTrajectory2DAsync<num>:: generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc, const Vector2 &lineDirection) noexcept {
    assert(lineDirection.length()!=0);// We need to give an input direction
    double rotation=lineDirection.angle();
    generateTrajectory(Vector2(0,0), initialVel.rotate(-rotation),(finalPos-initialPos).rotate(-rotation), maxVel, maxAcc,1.3);

}
}
