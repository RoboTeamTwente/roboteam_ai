//
// Created by rolf on 14/12/18.
//

#ifndef ROBOTEAM_AI_DRIBBLEROTATE_H
#define ROBOTEAM_AI_DRIBBLEROTATE_H

#include "Skill.h"

namespace rtt::ai {

class DribbleRotate : public Skill {
   private:
    enum Progression { ROTATING, SUCCESS, FAIL };
    Progression currentProgression;
    void checkProgression();
    Angle targetAngle;
    double startAngle, dir;

   public:
    explicit DribbleRotate(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIBBLEROTATE_H
