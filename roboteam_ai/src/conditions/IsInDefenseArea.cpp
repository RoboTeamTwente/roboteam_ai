//
// Created by rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"

namespace rtt{
namespace ai {
  IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {}

  bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double margin) {
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
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

  bt::Node::Status IsInDefenseArea::Update() {
    bool ourDefenseArea;
    if (blackboard->HasBool("ourDefenseArea")) {
        ourDefenseArea = blackboard->GetBool("ourDefenseArea");
    } else {
        ourDefenseArea = true;
    }

    roboteam_msgs::World world = World::get_world();
    Vector2 ballPos(world.ball.pos);

    double margin = 0.0;
    if (blackboard->HasDouble("margin")) {
        margin = blackboard->GetDouble("margin");
    }

    Vector2 point = ballPos;

    // If robot pos should be checked instead of ball pos, get my position and use that as the point.
    if (blackboard->HasBool("robot") && blackboard->GetBool("robot")) {
        int robotID = blackboard->GetInt("ROBOT_ID");
        boost::optional<roboteam_msgs::WorldRobot> findBot = World::getWorldBot(robotID, true);
        roboteam_msgs::WorldRobot me;
        if (findBot) {
            me = *findBot;
            point = me.pos;
        } else {
            return Status::Invalid;
        }
    }

    // Do the check
    if (isWithinDefenseArea(ourDefenseArea, point, margin)) {
        return Status::Success;
    }
    return Status::Failure;
  }
}
}
