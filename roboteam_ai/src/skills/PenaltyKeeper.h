//
// Created by rolf on 23-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYKEEPER_H
#define ROBOTEAM_AI_PENALTYKEEPER_H
#include "Skill.h"
#include <roboteam_ai/src/control/BasicPosControl.h>
namespace rtt{
namespace ai{
class PenaltyKeeper : public Skill  {
    private:
        // three states, one for waiting until they kick, one for intercepting the kick and one for after the kick
        enum PenaltyState {WAITING,BALLSHOT};
        PenaltyState state;
        Vector2 firstBallPos;
        int ballNotShotTicks;
        std::pair<Vector2,Vector2> goalLine;
        Vector2 computeDefendPos();
        Vector2 interceptBallPos();
        std::pair<Vector2,Vector2> getGoalLine();
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
        explicit PenaltyKeeper(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

};
}
}


#endif //ROBOTEAM_AI_PENALTYKEEPER_H
