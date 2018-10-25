//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {
  IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

  }

  bt::Node::Status IsInDefenseArea::Update() {

    bool ourDefenseArea;
    if (blackboard->HasBool("ourDefenseArea")) {
        ourDefenseArea = blackboard->GetBool("ourDefenseArea");
    } else {
        ourDefenseArea = true;
    }

    double margin = 0.0;
    if (blackboard->HasDouble("margin")) {
        margin = blackboard->GetDouble("margin");
    }

    roboteam_msgs::World world = World::get_world();
    Vector2 ballPos(world.ball.pos);
    Vector2 point = ballPos;

    // If robot pos should be checked instead of ball pos, get my position and use that as the point.
    if (blackboard->HasBool("robot") && blackboard->GetBool("robot")) {
        int robotID = blackboard->GetInt("ROBOT_ID");
        boost::optional<roboteam_msgs::WorldRobot> findBot = World::getRobotForId(robotID, true);
        roboteam_msgs::WorldRobot me;
        if (findBot) {
            me = *findBot;
            point = me.pos;
        } else {
            return Status::Invalid;
        }
    }

    if (Field::pointIsInDefenceArea(point, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
  }

} // ai
} // rtt
