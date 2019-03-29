//
// Created by rolf on 5-3-19.
//

#ifndef ROBOTEAM_AI_COACHDEFEND_H
#define ROBOTEAM_AI_COACHDEFEND_H
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
#include "../control/positionControllers/NumTreePosControl.h"
namespace rtt{
namespace ai{
class CoachDefend : public Skill{
    public:
        explicit CoachDefend(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        bt::Node::Status onUpdate() override;
        void onTerminate(bt::Node::Status) override;
    private:
        control::NumTreePosControl gtp;
};
}
}


#endif //ROBOTEAM_AI_COACHDEFEND_H
