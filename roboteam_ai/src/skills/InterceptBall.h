//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_INTERCEPTBALL_H
#define ROBOTEAM_AI_INTERCEPTBALL_H

#include "Skill.h"
#include "roboteam_ai/src/control/PIDController.h"

namespace rtt {
namespace ai {

class InterceptBall :public Skill {
    private:
        enum Progression {
          INTERCEPTING, CLOSETOPOINT, INPOSITION, BALLDEFLECTED, BALLMISSED
        };
        Progression currentProgression;
        void checkProgression();

        void sendInterceptCommand();
        void sendFineInterceptCommand();
        void sendStopCommand();

        bool missedBall(Vector2 startBall, Vector2 endBall, Vector2 ballVel);
        bool ballDeflected();

        control::PositionController goToPos;

        Vector2 ballStartPos, ballStartVel, ballEndPos, interceptPos;
        Vector2 deltaPos;
        int tickCount, maxTicks;
        control::PIDController pid,finePid;
        bool backwards;

        // Relevant to keeper only
        bool keeper;
        bool ballToGoal();
        bool ballInGoal();

        //Interface
        std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;

    public:
        explicit InterceptBall(string name, bt::Blackboard::Ptr blackboard);
        void sendMoveCommand(Vector2 targetPos);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

        Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall);

};

}
}

#endif //ROBOTEAM_AI_INTERCEPTBALL_H
