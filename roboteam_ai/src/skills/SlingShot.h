//
// Created by rolf on 11-4-19.
//

#ifndef ROBOTEAM_AI_SLINGSHOT_H
#define ROBOTEAM_AI_SLINGSHOT_H

#include "Skill.h"
#include "../control/positionControllers/BasicPosControl.h"
namespace rtt{
namespace ai{
class SlingShot : public Skill {
    private:
        //constants
        const int maxDribbleTicks = 30;
        const int maxWaitingTicks = 10;

        enum Progression{FAIL,DRIBBLING,ROTATINGAWAY,WAITINGFORRESULT,SUCCESS};
        Progression progression;
        int dribbledTicks=0;
        int waitingTicks=0;
        int ballShotTicks=0;

        Vector2 kickPos;
        double kickOrient;
        double rotateAngle;
        control::BasicPosControl gtp;
        Progression updateProgress(Progression currentProgress);
        bool robotAtAngle();
        bool ballShot();
        void setRotate();
        void sendDribbleCommand();
        void sendRotateCommand();
        void sendWaitCommand();

    public:
        explicit SlingShot(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
};
}
}


#endif //ROBOTEAM_AI_SLINGSHOT_H
