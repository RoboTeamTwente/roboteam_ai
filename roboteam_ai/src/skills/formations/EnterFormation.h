//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_ENTERFORMATION_H
#define ROBOTEAM_AI_ENTERFORMATION_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "roboteam_ai/src/skills/Skill.h"
#include "gtest/gtest_prod.h"

namespace rtt {
namespace ai {

class EnterFormation : public Skill {
    FRIEND_TEST(FormationTest, formation_test);
public:
    explicit EnterFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
    void onTerminate(bt::Node::Status) override;

protected:
    control::NumTreePosControl gtp;
    double errorMargin = 0.1;
    static std::vector<std::shared_ptr<Robot>> robotsInFormation;
    int robotsInFormationMemory = 0;
    virtual Vector2 getFormationPosition();
    Vector2 targetLocation;

    void removeRobotFromFormation();
    void addRobotToFormation();
    bool robotIsInFormation();
    bool formationHasChanged();
    bool robotIsInPosition();
    void updateFormation();
    void moveToTarget();
    void setFinalAngle();
};

}
}
#endif //ROBOTEAM_AI_ENTERFORMATION_H
