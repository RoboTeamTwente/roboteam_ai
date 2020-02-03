//
// Created by baris on 15-1-19.
//

#ifndef ROBOTEAM_AI_BASICGOTOPOS_H
#define ROBOTEAM_AI_BASICGOTOPOS_H

#include <coach/BallplacementCoach.h>
#include <control/PositionUtils.h>
#include <world/Field.h>
#include "GoToPos.h"
#include "interface/api/Input.h"
#include "skills/Skill.h"

namespace rtt::ai {

class GTPSpecial : public GoToPos {
   private:
    enum Type { goToBall, ballPlacementBefore, ballPlacementAfter, getBallFromSide, defaultType, freeKick, getBackIn, ourGoalCenter, ourDefenseAreaCenter };
    Type type;
    Type stringToType(const std::string &string);

   public:
    explicit GTPSpecial(std::string name, bt::Blackboard::Ptr blackboard);
    void gtpInitialize() override;
    Status gtpUpdate() override;
    void gtpTerminate(Status s) override;

    double getballFromSideMargin = 0.3;
    Vector2 getBallFromSideLocation();
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BASICGOTOPOS_H
