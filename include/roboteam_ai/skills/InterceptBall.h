//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_INTERCEPTBALL_H
#define ROBOTEAM_AI_INTERCEPTBALL_H

#include "Skill.h"

namespace rtt::ai {

class InterceptBall : public Skill {
   private:
    const double BALL_DEFLECTION_ANGLE = 30.0 / 180.0 * M_PI;    // Angle at which a ball is considered 'deflected'
    const double INTERCEPT_POSDIF = 0.015;                       // Meters acceptable deviation
    const double TURNING_DISTANCE = 0.025;                       // m distance at which we start turning towards the target angle (if desired)
    const double TURN_TIME = 0.3;                                // estimated time to make a large rotation (90 to 180 degrees) on the current robot
    const double GOAL_MARGIN = Constants::BALL_RADIUS() + 0.05;  // if the ball is shot next to the goal within this margin we still try to intercept
    enum Progression { INTERCEPTING, INPOSITION, BALLDEFLECTED, BALLMISSED };
    Progression currentProgression;
    void checkProgression();

    void sendStopCommand();

    bool missedBall(Vector2 startBall, Vector2 endBall, Vector2 ballVel);
    bool ballDeflected();

    Vector2 ballStartPos;
    Vector2 ballStartVel;
    Vector2 ballEndPos;
    Vector2 interceptPos;

    Vector2 deltaPos;
    int tickCount;
    int maxTicks;
    bool backwards;
    bool stayAtOrientation = false;
    bool orientationLocked = false;
    // Relevant to keeper only
    bool keeper;
    bool ballToGoal();
    bool ballInGoal();

   public:
    explicit InterceptBall(std::string name, bt::Blackboard::Ptr blackboard);
    void sendMoveCommand(Vector2 targetPos);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;

    Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall);
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_INTERCEPTBALL_H
