//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_ENTERFORMATION_H
#define ROBOTEAM_AI_ENTERFORMATION_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
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

private:
    control::NumTreePosControl gtp;
    double errorMargin = 0.1;
    enum Formation {
      Normal,
      Penalty,
      FreeKick
    };
    Formation formation = Normal;

    static std::vector<std::shared_ptr<Robot>> robotsInFormation;
    int robotsInFormationMemory = 0;
    Vector2 getFormationPosition();
    Vector2 targetLocation;
};

}
}
#endif //ROBOTEAM_AI_ENTERFORMATION_H
