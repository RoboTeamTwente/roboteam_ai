//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ballHandling/BallHandlePosControl.h>
#include "roboteam_ai/src/skills/Skill.h"

namespace rtt {
namespace ai {

class GoToPos : public Skill {
    protected:
        enum GoToType {
          basic,
          numTree
        };
        GoToType goToType;
        GoToType stringToGoToType(const std::string &gtt);
        void setPositionController(const GoToType &goToType);

        Vector2 targetPos = {};
        Angle targetAngle = 0.0;
        double maxVel;
        double errorMargin = Constants::GOTOPOS_ERROR_MARGIN();
        double angleErrorMargin = Constants::GOTOPOS_ANGLE_ERROR_MARGIN();

        std::shared_ptr<control::PosController> posController;

        virtual void gtpInitialize() = 0;
        virtual Status gtpUpdate() = 0;
        virtual void gtpTerminate(Status s) = 0;
    public:
        explicit GoToPos(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
