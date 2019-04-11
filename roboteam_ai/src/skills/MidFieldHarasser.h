//
// Created by thijs on 10-4-19.
//

#ifndef ROBOTEAM_AI_MIDFIELDHARASSER_H
#define ROBOTEAM_AI_MIDFIELDHARASSER_H

#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>

namespace rtt {
namespace ai {

class MidFieldHarasser : public Skill {
    private:
        control::NumTreePosControl goToPos;
        Vector2 targetPos;
        int zone = - 1;

        static int robotsInMemory;
        static vector<RobotPtr> midFieldHarassers;
    public:
        explicit MidFieldHarasser(std::string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
        Vector2 getHarassPosition();

        };

}
}

#endif //ROBOTEAM_AI_MIDFIELDHARASSER_H
