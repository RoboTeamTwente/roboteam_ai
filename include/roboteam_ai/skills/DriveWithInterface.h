//
// Created by baris on 10-5-19.
//

#ifndef ROBOTEAM_AI_DRIVEWITHINTERFACE_H
#define ROBOTEAM_AI_DRIVEWITHINTERFACE_H

#include "control/positionControl/PositionControl.h"
#include "Skill.h"

namespace rtt::ai {

class DriveWithInterface : public Skill {
   public:
    explicit DriveWithInterface(string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;

    private:
        control::PositionControl *numTreeGtp = nullptr;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIVEWITHINTERFACE_H
