//
// Created by robzelluf on 4/9/19.
//

#ifndef ROBOTEAM_AI_REFLECTKICK_H
#define ROBOTEAM_AI_REFLECTKICK_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "InterceptBall.h"
#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>

namespace rtt {
namespace ai {

class ReflectKick : public Skill {
private:
    control::NumTreePosControl numTreeGtp;
    control::BasicPosControl basicGtp;

    Vector2 goalTarget;
    Vector2 reflectionPos;
    Angle angleToGoalTarget;
    Angle angleToBall;

    Vector2 ballStartPos;
    Vector2 ballEndPos;
    Vector2 ballStartVel;

    void intercept();
    Vector2 computeInterceptPoint(const Vector2& startBall, const Vector2& endBall);
public:
    explicit ReflectKick(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
    Vector2 getFarSideOfGoal();
};

}
}


#endif //ROBOTEAM_AI_REFLECTKICK_H
