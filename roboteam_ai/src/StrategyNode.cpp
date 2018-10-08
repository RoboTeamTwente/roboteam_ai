#include "ros/ros.h"
#include "io/StrategyIOManager.h"
#include "DangerFinder/DangerFinder.h"

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");

  df::DangerData danger;
  io::StrategyIOManager strategyIOManager;
  ros::Rate rate(10);

  roboteam_msgs::World worldMsg;
  roboteam_msgs::GeometryData geometryMsg;

  df::DangerFinder::instance().start();

  while (ros::ok()) {
    ros::spinOnce();

    worldMsg = strategyIOManager.getWorldState();
    geometryMsg = strategyIOManager.getGeometryData();
    ai::World::set_world(worldMsg);
    ai::World::set_field(geometryMsg.field);

    danger = df::DangerFinder::instance().getMostRecentData();
    if (df::DangerFinder::instance().hasCalculated()) {

//      std::cout << "[ ";
//      for (unsigned i = 0; i < danger.flags.size(); i++) {
//        std::cout << std::to_string(danger.flags.at(i)) << ", ";
//      }
//      std::cout << "] " << std::endl;

    }


    strategyIOManager.getRoleFeedback();
    rate.sleep();
  }
  return 0;
}
