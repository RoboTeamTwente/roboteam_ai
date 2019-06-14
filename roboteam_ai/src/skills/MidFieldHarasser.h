//
// Created by thijs on 10-4-19.
//

#ifndef ROBOTEAM_AI_MIDFIELDHARASSER_H
#define ROBOTEAM_AI_MIDFIELDHARASSER_H

#include "Skill.h"

namespace rtt {
namespace ai {

class MidFieldHarasser : public Skill {
private:
    const double HARASSING_SAFETY_MARGINS = 1.0;
    Vector2 targetPos;
    int robotBeingHarassed;
    Vector2 getHarassTarget();
public:
    explicit MidFieldHarasser(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_MIDFIELDHARASSER_H
