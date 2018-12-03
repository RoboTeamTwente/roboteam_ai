//
// Created by baris on 22/10/18.
//

#include "IsInZone.h"

namespace rtt {
namespace ai {
IsInZone::IsInZone(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status IsInZone::update() {
    auto world = World::get_world();
    auto field = Field::get_field();

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
            ROS_WARN_STREAM("IsInZone: robot with this ID not found, ID: " << robotID);
            return Status::Failure;
        }
    }

    //Initialize Zone
    double zone_x1 = 0.0;
    double zone_x2 = 0.0;
    double zone_y1 = 0.0;
    double zone_y2 = 0.0;

    //Check for zone number and set zone accordingly
    if (properties->hasInt("zone")) {

        switch (properties->getInt("zone")) {

            // zone 1: is as seen from the goal -> left rear
        case 1:zone_x1 = - 4.5;
            zone_x2 = - 1.5;
            zone_y1 = 3.0;
            zone_y2 = 0.0;
            break;

            // zone 2: is as seen from the goal -> right rear
        case 2:zone_x1 = - 4.5;
            zone_x2 = - 1.5;
            zone_y1 = 0.0;
            zone_y2 = - 3.0;
            break;

            // zone 3: all field except area close to opponents goal
        case 3:zone_x1 = - 4.5;
            zone_x2 = 3.5;
            zone_y1 = - 3.0;
            zone_y2 = 3.0;
            break;

            // zone 4: everywhere except the edges of the field
            // (where the robot cant get the ball in case of demo field)
        case 4:zone_x1 = - field.field_length/2 + 0.2;
            zone_x2 = field.field_length/2 - 0.2;
            zone_y1 = - field.field_width/2 + 0.2;
            zone_y2 = field.field_width/2 - 0.2;
            break;

            // zone 5: zone 4 with larger margin
        case 5:zone_x1 = - field.field_length/2 + 0.35;
            zone_x2 = field.field_length/2 - 0.35;
            zone_y1 = - field.field_width/2 + 0.35;
            zone_y2 = field.field_width/2 - 0.35;
            break;

            // TODO: find out why is this what it is
        case 6:zone_x1 = - 4.8;
            zone_x2 = - 4.0;
            zone_y1 = - 1.5;
            zone_y2 = 1.5;
            break;

        default:ROS_WARN_STREAM("Not valid zone Int " << properties->getInt("zone"));
            break;
        }
    }

    // TODO: BB names
    // Set zone according to points given in blackboard if they are a thing
    if (properties->hasDouble("x1")) { zone_x1 = properties->getDouble("x1"); }
    if (properties->hasDouble("x2")) { zone_x2 = properties->getDouble("x2"); }
    if (properties->hasDouble("y1")) { zone_y1 = properties->getDouble("y1"); }
    if (properties->hasDouble("y2")) { zone_y2 = properties->getDouble("y2"); }

    // The condition that decides the status
    if (point.x > zone_x1 && point.x < zone_x2 && point.y > zone_y1 && point.y < zone_y2) {
        return Status::Success;
    }
    return Status::Failure;
}
} //ai
} //rtt