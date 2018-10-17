//
// Created by rolf on 17-10-18.
//

//TODO: Implement BB variables
//TODO: Implement getWorldBot() (see tactics utils/utils.cpp)
#include "IsInDefenseArea.hpp"

namespace rtt{
namespace ai {

    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double margin) {

        roboteam_msgs::GeometryFieldSize field = World::get_field();
        double xBound;
        double yTopBound;
        double yBottomBound;
        if (ourDefenseArea) {
            xBound = field.left_penalty_line.begin.x;
            if (field.left_penalty_line.begin.y < field.left_penalty_line.end.y) {
                yBottomBound = field.left_penalty_line.begin.y;
                yTopBound = field.left_penalty_line.end.y;
            } else {
                yBottomBound = field.left_penalty_line.end.y;
                yTopBound = field.left_penalty_line.begin.y;
            }
            // yTopBound = field.top_left_penalty_stretch.begin.y;
            // yBottomBound = field.bottom_left_penalty_stretch.begin.y;
            if (point.x < (xBound + margin) && point.y<(yTopBound + margin) && point.y>(yBottomBound - margin)) {
                return true;
            } else return false;
        } else { // their defense area
            xBound = field.right_penalty_line.begin.x;
            if (field.right_penalty_line.begin.y < field.right_penalty_line.end.y) {
                yBottomBound = field.right_penalty_line.begin.y;
                yTopBound = field.right_penalty_line.end.y;
            } else {
                yBottomBound = field.right_penalty_line.end.y;
                yTopBound = field.right_penalty_line.begin.y;
            }
            if (point.x > (xBound - margin) && point.y<(yTopBound + margin) && point.y>(yBottomBound - margin)) {
                return true;
            } else return false;
        }

    }

    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point) {
        return isWithinDefenseArea(ourDefenseArea, point, 0.0);
    }

    IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {}

    bt::Node::Status IsInDefenseArea::Update() {

        bool ourDefenseArea;
        if (HasBool("ourDefenseArea")) {
            ourDefenseArea = GetBool("ourDefenseArea");
        } else {
            ourDefenseArea = true;
        }

        roboteam_msgs::World world = World::get_world();
        Vector2 ballPos(world.ball.pos);

        double margin = 0.0;
        if (HasDouble("margin")) {
            margin = GetDouble("margin");
        }

        Vector2 point = ballPos;

        // If robot pos should be checked instead of ball pos, get my position and use that as the point.
        if (HasBool("robot") && GetBool("robot")) {
            int robotID = GetInt("ROBOT_ID");
            boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
            roboteam_msgs::WorldRobot me;
            if (findBot) {
                me = *findBot;
            } else {
                ROS_WARN_STREAM("IsInDefenseArea: robot with this ID not found, ID: " << robotID);
                return Status::Invalid;
            }
            point = me.pos;
        }

        // Do the check
        if (isWithinDefenseArea(ourDefenseArea, point, margin)) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
}
}