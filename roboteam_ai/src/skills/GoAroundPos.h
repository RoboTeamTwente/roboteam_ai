//
// Created by rolf on 30-1-19.
//

#ifndef ROBOTEAM_AI_GOAROUNDPOS_H
#define ROBOTEAM_AI_GOAROUNDPOS_H
#include "Skill.h"
namespace rtt {
namespace ai {

class GoAroundPos : public Skill {
        enum Progression{FAIL,ROTATING,STOPPING,DONE};
        Progression currentProgress;
        bool ballIsTarget;
        Vector2 targetPos,deltaPos,commandPos;
        double startAngle,endAngle,angleDif;
        double distanceFromPoint;
        double distanceError;
        int rotateDir;
        double rotatingSpeed; //rad/s
        double currentTick,maxTick;
        void sendRotateCommand();

        Progression checkProgression();
    public:
        explicit GoAroundPos(string name,bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
        std:: string node_name() override {return "GoAroundPos";};

};

}//ai
}//rtt
#endif //ROBOTEAM_AI_GOAROUNDPOS_H
