//
// Created by baris on 10-5-19.
//

#ifndef ROBOTEAM_AI_DRIVEWITHINTERFACE_H
#define ROBOTEAM_AI_DRIVEWITHINTERFACE_H

#include "Skill.h"

namespace rtt {
namespace ai {

class DriveWithInterface : public Skill {
    public:
        explicit DriveWithInterface(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
};
}
}

#endif //ROBOTEAM_AI_DRIVEWITHINTERFACE_H
