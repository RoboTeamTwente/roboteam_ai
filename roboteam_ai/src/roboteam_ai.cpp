#include "ros/ros.h"
#include "io/StrategyIOManager.h"
#include "DangerFinder/DangerFinder.h"
#include "io/IOManager.h"
#include "treeinterp/TreeInterpreter.h"

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;

roboteam_msgs::World worldMsg;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "StrategyNode");

    // init IOManager and subscribe to all topics immediately
    io::IOManager IOManager(true);
    ros::Rate rate(10);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;

    /*
     * what we will do:
     *
     * We will loop through frames.
     * each frame we will decide on a strategy (tick through strat tree)
     * based on that strategy we will tick through role tree
     *
     */

    while (ros::ok()) {
        ros::spinOnce();

        worldMsg = IOManager.getWorldState();
        geometryMsg = IOManager.getGeometryData();
        ai::World::set_world(worldMsg);
        ai::Field::set_field(geometryMsg.field);

        bt::BehaviorTree strategyTree = TreeInterpreter::getInstance().getTreeWithID("ai_project", "id of the tree");
        strategyTree.Tick();


        rate.sleep();
    }
    return 0;
}
