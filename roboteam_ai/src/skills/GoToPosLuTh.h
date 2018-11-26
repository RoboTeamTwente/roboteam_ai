//
// Created by thijs on 24-11-18.
//
#include "Skill.h"
#include <cmath>

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_H
#define ROBOTEAM_AI_GOTOPOSLUTH_H

namespace rtt {
namespace ai {
class GoToPosLuTh : public Skill {

    private:

        using Status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;

        bool goToBall;

        enum Progression {
          ON_THE_WAY, DONE, FAIL, INVALID
        };
        Progression currentProgress;
        Progression checkProgression();

        Vector2 targetPos;

        bool checkTargetPos(Vector2 pos);
        void sendMoveCommand();
        bool calculateNumericDirection(float &x_vel, float &y_vel, float &angle);
    public:

        explicit GoToPosLuTh(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;

        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

};
} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
