//
// Created by thijs on 19-11-18.
//

#include "Skill.h"

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_OLD_H
#define ROBOTEAM_AI_GOTOPOSLUTH_OLD_H

namespace rtt {
namespace ai {
class SkillGoToPos : public Skill {

    private:

        GoToType goToType;
        bool goToBall;
        Vector2 targetPos;
        control::ControlGoToPos goToPos;

        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();


    public:
        explicit SkillGoToPos(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
