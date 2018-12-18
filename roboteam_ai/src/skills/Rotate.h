//
// Created by thijs on 24-10-18.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDPOINT_H
#define ROBOTEAM_AI_ROTATEAROUNDPOINT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Rotate : public Skill {
    private:
        bool rotateToBall;
        int rotateToRobotID;
        bool rotateToEnemyGoal;
        bool rotateToOurGoal;
        bool robotIsEnemy = false;
        enum Progression {
          ROTATING, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();
        double deltaAngle;
        double targetAngle;
    public:
        explicit Rotate(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
