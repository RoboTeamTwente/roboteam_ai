//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_ENTERFORMATION_H
#define ROBOTEAM_AI_ENTERFORMATION_H

#include "Skill.h"

namespace rtt {
namespace ai {

class EnterFormation : public Skill {
public:
    explicit EnterFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
    void onTerminate(bt::Node::Status) override;

private:
    control::PositionController gtp;

    static std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> robotsInFormation;
    int robotsInFormationMemory = 0;
    Vector2 getFormationPosition();
    Vector2 targetLocation;
};

}
}
#endif //ROBOTEAM_AI_ENTERFORMATION_H
