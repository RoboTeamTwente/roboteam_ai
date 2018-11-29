//
// Created by rolf on 21/11/18.
//

#ifndef ROBOTEAM_AI_ROTATETOANGLE_H
#define ROBOTEAM_AI_ROTATETOANGLE_H
#include "Skill.h"
namespace rtt{
namespace ai{
class RotateToAngle: public Skill {
    private:

        using Status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        double targetAngle,deltaAngle;
        bool useAngle;
        enum Progression {
          ROTATING, DONE, FAIL, INVALID
        };
        Progression currentProgress;
        Progression checkProgression();

    public:
        explicit RotateToAngle(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;

        void initialize() override;
        Status update() override;
        void terminate(Status s) override;


};
}//ai
}//rtt


#endif //ROBOTEAM_AI_ROTATETOANGLE_H
