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
        unit type;

    public:
        explicit GoBehindBall(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

};

}
}

#endif //ROBOTEAM_AI_GOBEHINDBALL_H
