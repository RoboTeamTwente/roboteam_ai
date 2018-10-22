//
// Created by baris on 22/10/18.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDPOINT_H
#define ROBOTEAM_AI_ROTATEAROUNDPOINT_H

#include "GoToPos.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"
#include "Skill.h"



namespace rtt {
namespace ai {

class RotateAroundPoint : public Skill {
public:

    RotateAroundPoint(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
    Status checkAndSetArguments();
    void stoprobot(int RobotID);
    std::string node_name() { return "RotateAroundPoint"; }

private:
    bt::Blackboard::Ptr goto_bb;
    uint32_t prevworldseq;
    bool firstworld=true;
    int robotID;
    double targetAngle;
    roboteam_msgs::WorldRobot robot;
    roboteam_msgs::WorldBall ball;
    Vector2 faceTowardsPos;
    double rotw;
    Vector2 center;
    double radius;
    GoToPos goToPos;
    double rotPconstant;
    double radiusPconstant;
    double turnPconstant;
} ;

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
} // ai
} // rtt
