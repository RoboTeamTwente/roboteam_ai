//
// Created by rolf on 14-1-19.
//

#ifndef ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
#define ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H

#include "Condition.h"
namespace rtt{
namespace ai{
class BallInDefenseAreaAndStill : public Condition{
    private:
        int currentTick;
        int maxTick;
        bool theirDefenceArea;

    public:
        explicit BallInDefenseAreaAndStill(std::string name = "BallInDefenseAreaAndStill", bt::Blackboard::Ptr blackboard = nullptr);
        void initialize() override;
        Status update() override;
        std::string node_name() override;
};
}
}


#endif //ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
