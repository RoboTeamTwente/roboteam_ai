//
// Created by rolf on 15-6-19.
//

#ifndef ROBOTEAM_AI_RESUMEPLAYAFTERPENALTY_H
#define ROBOTEAM_AI_RESUMEPLAYAFTERPENALTY_H
#include "Condition.h"
namespace rtt{
namespace ai{
class ResumePlayAfterPenalty : public Condition{
    private:
        int ticks=0;
        bool ballShot=false;
    public:
        explicit ResumePlayAfterPenalty(std::string name = "ResumePlayAfterPenalty", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
};
}
}


#endif //ROBOTEAM_AI_RESUMEPLAYAFTERPENALTY_H
