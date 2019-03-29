//
// Created by robzelluf on 12/7/18.
//

#ifndef ROBOTEAM_AI_DEFENDONROBOT_H
#define ROBOTEAM_AI_DEFENDONROBOT_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class DefendOnRobot : public Skill {
    private:
        double angleBetweenRobots;
        Vector2 newPosition;
        int pickOpponentToCover();
        static std::map<int, int> defencePairs;
        control::NumTreePosControl goToPos;
    int amountOfCycles{};
    protected:
        enum Progression {
          IDLE, DONE, FAIL
        };
        Progression currentProgress;
    public:
        explicit DefendOnRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        void onTerminate(Status s) override;
        Status onUpdate() override;
        Vector2 calculateLocation();
        int opponentWithBallID;
        int opponentToCoverID;
        RobotPtr opponentWithBall;
        RobotPtr opponentToCover;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_DEFENDONROBOT_H
