//
// Created by baris on 5-12-18.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Pass : public Skill {
    private:
        double errorMargin = 0.3;
        bool IamNumber1;
        int otherRobotID;
        double dBehindball = 0.25;
        GoToType goToType;
        std::shared_ptr<roboteam_msgs::WorldRobot> otherRobot;
        enum FSM012 {
          S1 = 1,
          S2 = 2,
          S3 = 3,
          S4 = 4,
          S5 = 5,
          S6 = 6,
          S7 = 7,
          S8 = 8,
          S9 = 9,
          S10 = 10,
          S11 = 11,
          S12 = 12
        };
        FSM012 fsm = S1;
        FSM012 fsmPlusPlus(FSM012 fsm);
        void receiveBall(roboteam_msgs::RobotCommand &command, const Vector2 &pos);
        void shootBall(roboteam_msgs::RobotCommand &command);
        void getBall();
        using PassState = Coach::PassState;
        control::ControlGoToPos goToPos;
        Vector2 deltaPos;
        Vector2 targetPos;

        Vector2 P1 = {- 1.4, 0.95};
        Vector2 P2 = {- 1.4, - 0.95};
        Vector2 P3 = {1.4, 0.95};

        bool amIClosest;
        bool newTarget;
        bool defensive;
        int robotToPass;
    public:
        explicit Pass(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_PASS_H
