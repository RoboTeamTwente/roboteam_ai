//
// Created by rolf on 14/12/18.
//

#ifndef ROBOTEAM_AI_DRIBBLEROTATE_H
#define ROBOTEAM_AI_DRIBBLEROTATE_H
#include "Skill.h"
namespace rtt{
namespace ai{
class DribbleRotate : public Skill{
    private:
        enum Progression{
          ROTATING,SUCCESS,FAIL
        };
        Progression currentProgression;
        void checkProgression();
        double startAngle,targetAngle, maxSpeed,incrementAngle,currentAngle,dir;
        int currentTick, maxTick,extraTick;
        bool rotateToGoal;
        double computeCommandAngle();
        bool robotHasBall(double frontRange);
    public:
        explicit DribbleRotate(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
        void sendMoveCommand();

};

}
}


#endif //ROBOTEAM_AI_DRIBBLEROTATE_H
