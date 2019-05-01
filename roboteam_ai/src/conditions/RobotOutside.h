//
// Created by baris on 1-5-19.
//

#ifndef ROBOTEAM_AI_ROBOTOUTSIDE_H
#define ROBOTEAM_AI_ROBOTOUTSIDE_H

#include "Condition.h"
namespace rtt{
namespace ai {



class RobotOutside : public Condition{

    public:
        explicit RobotOutside(std::string name = "RobotOutside", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
    private:
        bool checkPoint();

};

}
}

#endif //ROBOTEAM_AI_ROBOTOUTSIDE_H
