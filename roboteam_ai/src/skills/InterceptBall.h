//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_INTERCEPTBALL_H
#define ROBOTEAM_AI_INTERCEPTBALL_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include "roboteam_ai/src/control/PIDController.h"

namespace rtt {
namespace ai {

class InterceptBall :public Skill {
    private:

        const double BALL_DEFLECTION_ANGLE = 30.0/180.0*M_PI;    // Angle at which a ball is considered 'deflected'
        const double INTERCEPT_POSDIF = 0.04;    // Meters acceptable deviation

        const double INTERCEPT_P = 5.7;
        const double INTERCEPT_I = 1.7;
        const double INTERCEPT_D = 0.0;

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

        control::NumTreePosControl numtreeGTP;
        control::BasicPosControl basicGTP;

        Vector2 ballStartPos, ballStartVel, ballEndPos, interceptPos;
        Vector2 deltaPos;
        int tickCount, maxTicks;
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
