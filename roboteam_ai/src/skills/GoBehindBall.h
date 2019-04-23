//
// Created by baris on 21-2-19.
//

#ifndef ROBOTEAM_AI_GOBEHINDBALL_H
#define ROBOTEAM_AI_GOBEHINDBALL_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
#include "../world/Field.h"

namespace rtt {
namespace ai {
class GoBehindBall : public Skill {

    private:
        enum unit {
          penalty,
          freeKick,
          corner
        };
        control::NumTreePosControl goToPos;
        unit unitType;
        unit stringToUnit(std::string string);
        const double errorMargin = Constants::ROBOT_RADIUS() + 0.05;
        void publishCommand(Vector2 point, Vector2 velocity);

    public:
        explicit GoBehindBall(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

};

}
}

#endif //ROBOTEAM_AI_GOBEHINDBALL_H
