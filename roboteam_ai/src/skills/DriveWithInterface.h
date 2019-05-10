//
// Created by baris on 10-5-19.
//

#ifndef ROBOTEAM_AI_DRIVEWITHINTERFACE_H
#define ROBOTEAM_AI_DRIVEWITHINTERFACE_H

#include <roboteam_ai/src/control/positionControllers/PosController.h>
#include "Skill.h"
namespace rtt {
namespace ai {

class DriveWithInterface : public Skill {
    public:
        explicit DriveWithInterface(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

    private:
        std::shared_ptr<control::PosController> goToPos;


};
}
}

#endif //ROBOTEAM_AI_DRIVEWITHINTERFACE_H
