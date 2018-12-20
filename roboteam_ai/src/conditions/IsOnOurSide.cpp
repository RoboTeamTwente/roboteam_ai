//
// Created by robzelluf on 10/25/18.
//

#include "IsOnOurSide.h"

namespace rtt {
namespace ai {

IsOnOurSide::IsOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status IsOnOurSide::update() {
    roboteam_msgs::World world = World::get_world();
    roboteam_msgs::GeometryFieldSize field = rtt::ai::Field::get_field();

    double zone_x1, zone_x2, zone_y1, zone_y2;
    zone_x1 = - field.field_length/2;
    zone_x2 = 0;
    zone_y1 = - field.field_width/2;
    zone_y2 = field.field_length/2;

    // We assume that we are looking for a ball by default
    Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);
    Vector2 point = ballPos;

    // If we are looking for a robot we should change point
    if (properties->hasBool("robot") && properties->getBool("robot")) {
        int robotID = properties->getInt("ROBOT_ID");
        auto findBot = World::getRobotForId(robotID, true);//TODO: change the bool, was this way in the old code

        if (findBot) {
            point = (*findBot).pos;
        }
        else {
            return Status::Waiting;
        }
    }

    // The condition that decides the status
    if (point.x > zone_x1 && point.x < zone_x2 && point.y > zone_y1 && point.y < zone_y2) {
        return Status::Success;
    }
    return Status::Failure;
}

} //ai
} //rtt
