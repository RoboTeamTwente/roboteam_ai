//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_RECEIVE_H
#define ROBOTEAM_AI_RECEIVE_H

#include "Skill.h"

namespace rtt::ai {

class Receive : public Skill {
   private:
    bool readyToPassSet = false;
    bool canMoveInDefenseArea = false;

    Vector2 ballStartPos;
    Vector2 ballEndPos;
    Vector2 ballStartVel;

    Vector2 targetPos;

    virtual bool isInPosition(const Vector2 &behindTargetPos = {0, 0});

   protected:
    bool passFailed();

   public:
    explicit Receive(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
    Vector2 computeInterceptPoint(const Vector2 &startBall, const Vector2 &endBall);
    void intercept();
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_RECEIVE_H
