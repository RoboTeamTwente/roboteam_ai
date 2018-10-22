//
// Created by baris on 22/10/18.
//

#ifndef ROBOTEAM_AI_AIMAT_H
#define ROBOTEAM_AI_AIMAT_H


#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/Field.h"
#include "Skill.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "RotateAroundPoint.h"
#include <vector>
#include <string>

namespace rtt{
namespace ai {

class AimAt : public Skill {
public:
    AimAt(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    bt::Node::Status Update();
    std::string node_name() { return "AimAt"; }

    static std::vector<std::string> valid_options(const std::string& key) {
        if (key == "At") {
            return std::vector<std::string>({"robot", "ourgoal", "theirgoal"});
        }
        return std::vector<std::string>();
    }
private:
    RotateAroundPoint rotateAroundPoint;

};

#endif //ROBOTEAM_AI_AIMAT_H
} // ai
} // rtt