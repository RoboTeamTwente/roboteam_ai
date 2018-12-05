//
// Created by thijs on 24-10-18.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDPOINT_H
#define ROBOTEAM_AI_ROTATEAROUNDPOINT_H

#include "Skill.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {

class Rotate : public Skill {
    private:

        bool rotateToBall;
        int rotateToRobotID;
        bool rotateToEnemyGoal;
        bool rotateToOurGoal;
        bool robotIsEnemy = false;
        roboteam_msgs::WorldRobot robot;


        enum Progression {
          ROTATING, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();

        double deltaAngle;
        double targetAngle;

    public:
        explicit Rotate(string name, bt::Blackboard::Ptr blackboard);

        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

        std::string node_name() override;


};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
