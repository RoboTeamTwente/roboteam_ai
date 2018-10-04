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


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");

  df::DangerData danger;
  io::StrategyIOManager strategyIOManager;
  ros::Rate rate(10);

  roboteam_msgs::World worldMsg;
  roboteam_msgs::GeometryData geometryMsg;

  df::DangerFinder::worldMsg = &worldMsg;
  df::DangerFinder::instance().start();

  while (ros::ok()) {
    ros::spinOnce();
    // give the dangerfinder a reference to worldstate
    worldMsg = strategyIOManager.getWorldState();
    geometryMsg = strategyIOManager.getGeometryData();

    // hack all data in like old times
    rtt::LastWorld::set(worldMsg);
    rtt::LastWorld::set_field(geometryMsg.field);

    danger = df::DangerFinder::instance().getMostRecentData();
    if (df::DangerFinder::instance().hasCalculated()) {
      std::cout << "[ ";
      for (unsigned i = 0; i < danger.scores.size(); i++) {
        std::cout << danger.scores.at(i) << ", ";
      }
      std::cout << "] " << std::endl;
      std::cout << "[ ";
      for (unsigned i = 0; i < danger.flags.size(); i++) {
        std::cout << std::to_string(danger.flags.at(i)) << ", ";
      }
      std::cout << "] " << std::endl;

    }
    strategyIOManager.getRoleFeedback();
    rate.sleep();
  }
  return 0;
}
