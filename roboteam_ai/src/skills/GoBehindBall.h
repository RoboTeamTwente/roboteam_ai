//
// Created by baris on 21-2-19.
//

#ifndef ROBOTEAM_AI_GOBEHINDBALL_H
#define ROBOTEAM_AI_GOBEHINDBALL_H

#include "Skill.h"

namespace rtt {
namespace ai {
class GoBehindBall : public Skill {

    private:

        enum unit {
          penalty,
          freeKick,
          corner
        };
        control::PositionController goToPos;
        unit unitType;
        unit stringToUnit(std::string string);
        // TODO maybe be smarter than Thijs
        double errorMargin = 0.06;
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
