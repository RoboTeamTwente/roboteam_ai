//
// Created by robzelluf on 4/9/19.
//

#ifndef ROBOTEAM_AI_REFLECTKICK_H
#define ROBOTEAM_AI_REFLECTKICK_H

#include <control/BasicPosControl.h>
#include "Skill.h"
#include <coach/PassCoach.h>
#include <coach/BallplacementCoach.h>

namespace rtt {
namespace ai {

class ReflectKick : public Skill {
private:
    constexpr static double TOWARDS_GOAL_FACTOR = 0.8;
    Vector2 goalTarget;
    Vector2 reflectionPos;
    double robotAngle;
    bool kicked = false;

    Vector2 ballStartPos;
    Vector2 ballEndPos;
    Vector2 ballStartVel;

    Vector2 ballReceiveVel;
    bool ballReceiveVelSet = false;

    void intercept();
    Vector2 computeInterceptPoint(const Vector2& startBall, const Vector2& endBall);
    Vector2 getKicker();
    double getAngle();
    bool ballDeflected();
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
