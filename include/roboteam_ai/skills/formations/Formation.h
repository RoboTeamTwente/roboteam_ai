//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_FORMATION_H
#define ROBOTEAM_AI_FORMATION_H

#include <include/roboteam_ai/control/numTrees/NumTreePosControl.h>
#include "include/roboteam_ai/skills/Skill.h"
#include "gtest/gtest_prod.h"

namespace rtt {
namespace ai {

class Formation : public Skill {
    FRIEND_TEST(FormationTest, formation_test);
public:
    explicit Formation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
    void onTerminate(bt::Node::Status) override;
    void terminate(bt::Node::Status) override;

protected:

    // these two always need to be overridden
    virtual Vector2 getFormationPosition() =0;
    virtual std::shared_ptr<std::vector<RobotPtr>> robotsInFormationPtr() =0;

    double errorMargin = 0.1;
    static std::vector<RobotPtr> robotsInFormation;
    static bool update;
    static int updateCount;
    int robotsInFormationMemory = 0;

    Vector2 targetLocation;

    void removeRobotFromFormation();
    void addRobotToFormation();
    bool robotIsInFormation();
    bool formationHasChanged();
    bool robotIsInPosition();
    virtual void updateFormation();
    void moveToTarget();
    virtual void setFinalAngle();
    bool updateCounter();
};

}
}
#endif //ROBOTEAM_AI_FORMATION_H
