/*
 * Pick a nice defensive location,
 * drive towards there and keep between the ball and the goal.
 */

#ifndef ROBOTEAM_AI_DEFEND_H
#define ROBOTEAM_AI_DEFEND_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
#include "gtest/gtest_prod.h"

namespace rtt {
namespace ai {

class Defend : public Skill {
    FRIEND_TEST(Defendtest, defend_test);

public:
    explicit Defend(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
    void onTerminate(bt::Node::Status) override;
private:
    control::NumTreePosControl gtp;
    Vector2 targetLocation;
    static std::vector<std::shared_ptr<Robot>> allDefenders;
    Vector2 getDefensivePosition();
    int allDefendersMemory = 0;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_DEFEND_H
