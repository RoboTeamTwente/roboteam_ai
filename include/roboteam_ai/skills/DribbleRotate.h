//
// Created by rolf on 14/12/18.
//

#ifndef ROBOTEAM_AI_DRIBBLEROTATE_H
#define ROBOTEAM_AI_DRIBBLEROTATE_H

#include "Skill.h"
#include "include/roboteam_ai/control/ballHandling/BallHandlePosControl.h"

namespace rtt {
namespace ai {
class DribbleRotate : public Skill {
    private:
        control::BallHandlePosControl ballHandlePosControl;

        const double WAIT_TIME = 0.2;     // Seconds
        const double MAX_SPEED = 0.5;     // Rad/second

        enum Progression {
          ROTATING, SUCCESS, FAIL
        };
        Progression currentProgression;
        void checkProgression();
        Angle targetAngle;
        double startAngle, maxSpeed, incrementAngle, currentAngle, dir;
        int currentTick, maxTick, extraTick;
        bool rotateToGoal;
        double computeCommandAngle();
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
