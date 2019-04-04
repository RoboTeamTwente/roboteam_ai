//
// Created by baris on 3-4-19.
//

#ifndef ROBOTEAM_AI_STOP_H
#define ROBOTEAM_AI_STOP_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
namespace rtt {
namespace ai {

class Stop : public Skill {
    public:
        explicit Stop(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
    private:
        int robotsInFormationMemory = 0;
        bool isActive = false;
        static Vector2 getOffensiveActivePoint();
        static Vector2 getDefensiveActivePoint();

        control::NumTreePosControl goToPos;
        Vector2 getFormationPosition();
        Vector2 targetLocation;
        static std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> robotsInFormation;
        static int defensiveOffensive;



};
}}

#endif //ROBOTEAM_AI_STOP_H
