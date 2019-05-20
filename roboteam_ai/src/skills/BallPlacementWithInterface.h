//
// Created by thijs on 15-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
#define ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H

#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/BallHandlePosControl.h>

namespace rtt {
namespace ai {

class BallPlacementWithInterface : public Skill {
    public:
        explicit BallPlacementWithInterface(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;

    private:
        control::BallHandlePosControl ballHandlePosControl;


};

}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
