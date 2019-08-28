//
// Created by robzelluf on 2/18/19.
//

#ifndef ROBOTEAM_AI_ISBALLCLOSETOBORDER_H
#define ROBOTEAM_AI_ISBALLCLOSETOBORDER_H

#include "Condition.h"
#include <include/roboteam_ai/utilities/Constants.h>
#include "include/roboteam_ai/world/Field.h"

namespace rtt {
namespace ai {

class IsBallCloseToBorder : public Condition {
private:
    double margin = Constants::CLOSE_TO_BORDER_DISTANCE();
    bool ballShouldLayStill = false;
public:
    explicit IsBallCloseToBorder(std::string name = "IsBallCloseToBorder", bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    Status onUpdate() override;
};

}
}

#endif //ROBOTEAM_AI_ISBALLCLOSETOBORDER_H
