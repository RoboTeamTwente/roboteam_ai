//
// Created by thijs on 6-7-19.
//
#include <roboteam_ai/src/skills/formations/Formation.h>

#ifndef ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
#define ROBOTEAM_AI_BALLPLACEMENTFORMATION_H

namespace rtt {
namespace ai {
class BallPlacementFormation : public Formation {
    public:
        explicit BallPlacementFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    private:
        Vector2 getFormationPosition() override;
        void updateFormation() override;
        void setFinalAngle() override;
        std::vector<std::vector<Vector2>> getStopPositions();
        std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
        static std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormation;
};
}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
