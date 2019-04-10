//
// Created by rolf on 10-4-19.
//

#ifndef ROBOTEAM_AI_INTERCEPTROBOT_HPP
#define ROBOTEAM_AI_INTERCEPTROBOT_HPP

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "roboteam_ai/src/control/positionControllers/BasicPosControl.h"

#include "Skill.h"
namespace rtt{
namespace ai{
class InterceptRobot : public Skill{
    private:
        control::BasicPosControl gtp;
    public:
        explicit InterceptRobot(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};

}
}

#endif //ROBOTEAM_AI_INTERCEPTROBOT_HPP
