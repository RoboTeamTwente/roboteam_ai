//
// Created by baris on 10-5-19.
//

#ifndef ROBOTEAM_AI_DRIVEWITHINTERFACE_H
#define ROBOTEAM_AI_DRIVEWITHINTERFACE_H

#include "Skill.h"
#include "control/positionControl/PositionControl.h"

namespace rtt::ai {

class DriveWithInterface : public Skill {
   public:
    explicit DriveWithInterface(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIVEWITHINTERFACE_H
