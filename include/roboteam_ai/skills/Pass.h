//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <coach/PassCoach.h>
#include <control/BasicPosControl.h>
#include <control/PositionUtils.h>
#include <control/numtrees/NumTreePosControl.h>
#include <control/shot-controllers/ShotController.h>
#include <utilities/Constants.h>
#include "Skill.h"

namespace rtt::ai {

class Pass : public Skill {
   protected:
    enum PassType { DEFAULT, DEFENSIVE, FREEKICK };

    PassType passType = DEFAULT;

    PassType stringToType(const std::string &type);

    const double CLOSE_ENOUGH_TO_BALL = 0.7;
    const double SUCCESSFUL_PASS_ANGLE = 0.6;

    bool forcePass = false;
    int fails = 0;
    int maxTries = -1;
    bool passInitialized = false;
    bool hasShot = false;
    RobotPtr robotToPassTo;
    Vector2 targetPos;
    virtual void initiatePass();
    bool didShootProperly();
    int robotToPassToID = -1;
    Vector2 getKicker();
    virtual void makeCommand();

   public:
    explicit Pass(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PASS_H
