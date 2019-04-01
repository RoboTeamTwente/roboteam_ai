//
// Created by thijs on 19-11-18.
//

#include "Skill.h"
#include "../control/positionControllers/PosController.h"

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_OLD_H
#define ROBOTEAM_AI_GOTOPOSLUTH_OLD_H

namespace rtt {
namespace ai {
class SkillGoToPos : public Skill {

    private:
        bool goToBall;
        Vector2 targetPos;
        std::shared_ptr<control::PosController> posController;

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

    void setPosController(const string &gTT);
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
