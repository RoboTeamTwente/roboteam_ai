//
// Created by baris on 25-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDFREEKICK_H
#define ROBOTEAM_AI_DEFENDFREEKICK_H

#include "Formation.h"
namespace rtt {
namespace ai {

class DefendFreeKick : public Formation {
public:
    explicit DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
private:
    Vector2 getFormationPosition() override;
    std::shared_ptr<vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
    static std::shared_ptr<vector<std::shared_ptr<Robot>>> robotsInFormation;
    static std::vector<Vector2> posses;
    void onTerminate(Status s) override;
    std::vector<Vector2> getDefendFreeKick(int number);
};
}
}
#endif //ROBOTEAM_AI_DEFENDFREEKICK_H
