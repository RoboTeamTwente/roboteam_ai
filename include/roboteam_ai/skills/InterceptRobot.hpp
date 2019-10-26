//
// Created by rolf on 10-4-19.
//

#ifndef ROBOTEAM_AI_INTERCEPTROBOT_HPP
#define ROBOTEAM_AI_INTERCEPTROBOT_HPP

#include <control/numTrees/NumTreePosControl.h>
#include "control/BasicPosControl.h"

#include "Skill.h"
namespace rtt{
namespace ai{
class InterceptRobot : public Skill{
    private:
      //  control::BasicPosControl gtp;
        Vector2 getInterceptPos(Robot robotToIntercept);
    public:
        explicit InterceptRobot(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;

};

}
}

#endif //ROBOTEAM_AI_INTERCEPTROBOT_HPP
