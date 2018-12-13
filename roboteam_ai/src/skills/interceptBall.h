//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_INTERCEPTBALL_H
#define ROBOTEAM_AI_INTERCEPTBALL_H
#include "Skill.h"
#include "../control/PID.h"
namespace rtt{
namespace ai{
class InterceptBall :public Skill {
    private:
        enum Progression{
        INTERCEPTING, CLOSETOPOINT,OVERSHOOT, INPOSITION, BALLDEFLECTED,BALLMISSED
        };
        Progression currentProgression;
        void checkProgression();

        roboteam_msgs::WorldBall ball;
        void sendInterceptCommand();
        void sendFineInterceptCommand();
        void sendStopCommand();

        bool missBall(Vector2 startBall,Vector2 endBall,Vector2 ballVel);
        bool ballDeflected();

        Vector2 ballStartPos,ballStartVel,ballEndPos,interceptPos;
        Vector2 deltaPos;
        int tickCount,maxTicks;
        Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall);
        control::PID pid,finePid;

        bool keeper;
        bool ballToGoal();
        bool ballInGoal();
    public:
        explicit InterceptBall(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;
        Status update() override;
        void initialize() override;
        void terminate(Status s) override;

};

}
}


#endif //ROBOTEAM_AI_INTERCEPTBALL_H
