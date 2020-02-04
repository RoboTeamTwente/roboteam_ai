//
// Created by rolf on 23-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYKEEPER_H
#define ROBOTEAM_AI_PENALTYKEEPER_H
#include <control/BasicPosControl.h>
#include <roboteam_utils/Line.h>
#include "Skill.h"

namespace rtt::ai {
class PenaltyKeeper : public Skill {
   private:
    // three states, one for waiting until they kick, one for intercepting the kick and one for after the kick
    enum PenaltyState { WAITING, BALLSHOT };
    PenaltyState state;
    Vector2 firstBallPos;
    int ballNotShotTicks;
    Line goalLine;
    Vector2 computeDefendPos();
    Vector2 interceptBallPos();
    Line getGoalLine();
    void sendWaitCommand();
    void sendInterceptCommand();
    control::BasicPosControl gtp;
    PenaltyState updateState(PenaltyState currentState);
    bool isBallShot();

    bool preparation;

    /*
    int ballShotTicks;
    Vector2 initialPos,initialVel;
     */
   public:
    explicit PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PENALTYKEEPER_H
