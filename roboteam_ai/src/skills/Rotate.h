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
        using status = bt::Node::Status;

        bool rotateToBall;
        int rotateToRobotID;
        bool rotateToEnemyGoal;
        bool rotateToOurGoal;
        bool robotIsEnemy = false;
        roboteam_msgs::WorldRobot robot;


        enum Progression {
          ROTATING, DONE, FAIL, INVALID
        };
        Progression currentProgress;
        Progression checkProgression();

        double deltaAngle;
        double targetAngle;

    public:
        explicit Rotate(string name, bt::Blackboard::Ptr blackboard);

        void Initialize() override;
        Status Update() override;
        void Terminate(status s) override;

        static double getAngularVelocity(double robotAngle, double targetAngle);
        std::string node_name() override;


};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
