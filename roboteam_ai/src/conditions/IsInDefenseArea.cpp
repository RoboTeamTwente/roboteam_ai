//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {
IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name,
        blackboard) { }

bt::Node::Status IsInDefenseArea::update() {
    bool ourDefenseArea = properties->hasBool("ourDefenseArea") ? properties->getBool("ourDefenseArea") : true;
    double margin = properties->hasDouble("margin") ? properties->getDouble("margin") : 0.0;

    roboteam_msgs::World world = World::get_world();
    Vector2 ballPos(world.ball.pos);
    Vector2 point = ballPos;

    // If robot pos should be checked instead of ball pos, get my position and use that as the point.
    if (properties->hasBool("robot") && properties->getBool("robot")) {
        int robotID = properties->getInt("ROBOT_ID");
        boost::optional<roboteam_msgs::WorldRobot> findBot = World::getRobotForId(robotID, true);
        roboteam_msgs::WorldRobot me;
        if (findBot) {
            me = *findBot;
            point = me.pos;
        }
        else {
            return Status::Failure;
        }
    }

    if (Field::pointIsInDefenceArea(point, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
