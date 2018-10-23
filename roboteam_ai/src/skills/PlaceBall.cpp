//
// Created by mrlukasbos on 22-10-18.
//

#include "PlaceBall.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

// Skills need to be combined at this point.
// we need getBall to get the ball
// And we need some kind of goToPos, backwards, with dribbling

bt::Node::Status PlaceBall::Update() {
  roboteam_msgs::WorldBall ball = World::getBall();
  unsigned int robotID = blackboard->GetInt("ROBOT_ID");
  auto robot = World::getRobotForId(robotID, true);
  if (!robot) {
    ROS_ERROR_STREAM("[PlaceBall] Could not find robot #" << robotID << " in world! Failing.");
    return Status::Failure;
  }
  return Skill::Update();
}

} // ai
} // rtt
