/*
 * Creates an instance of StrategyNode
 * The strategynode decides which strategy to use based upon the following game parameters:
 *  1) The world state, which contains:
 *    a) The refcommands
 *    b) The location of all robots (both us and them)
 *    c) the field geometry
 *
 *  2) A danger list which is generated based upon the world state
 *
 * RoboTeamTwente, september 2018
 */

#include "ros/ros.h"
#include "io/StrategyIOManager.h"
#include "DangerFinder/DangerFinder.h"

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;

roboteam_msgs::World worldMsg;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");

  df::DangerData danger;
  io::StrategyIOManager strategyIOManager;
  ros::Rate rate(10);


  df::DangerFinder::worldMsg = &worldMsg;
  df::DangerFinder::instance().start();

  // this must be called whenever a world message has been received....
  std::mutex dangerMutex;
  std::lock_guard<std::mutex> lock(dangerMutex);

  while (ros::ok()) {
    ros::spinOnce();
    // give the dangerfinder a reference to worldstate
    worldMsg = strategyIOManager.getWorldState();
    if (df::DangerFinder::instance().hasCalculated()) {

      for (unsigned i = 0; i < danger.dangerList.size(); i++) {
        int id = danger.dangerList.at(i);
        std::cout << "id " << id << std::endl;
      }
    }

    strategyIOManager.getRoleFeedback();

    rate.sleep();
  }
  return 0;
}
