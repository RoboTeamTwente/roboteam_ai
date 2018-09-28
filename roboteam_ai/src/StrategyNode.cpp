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
#include "danger_finder/DangerFinder.h"

namespace df = rtt::ai::dangerfinder;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");


  df::DangerData danger;
  std::mutex dangerMutex;

  StrategyIOManager strategyIOManager;


  // this must be called whenever a world message has been received...
  std::lock_guard<std::mutex> lock(dangerMutex);
  danger = df::DangerFinder::instance().getMostRecentData();

//  if (df::DangerFinder::instance().hasCalculated()) {
//    for (unsigned i = 0; i < danger.dangerList.size(); i++) {
//      int id = danger.dangerList.at(i);
//      auto bot = botWithId(id, msg.them);
//      if (bot) {
//        msg.dangerList.push_back(*bot);
//        msg.dangerScores.push_back(danger.scores.at(id));
//        msg.dangerFlags.push_back(danger.flags.at(id));
//      }
//    }
//  }


  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    strategyIOManager.getWorldState();
    strategyIOManager.getRoleFeedback();

    rate.sleep();
  }
  return 0;
}
