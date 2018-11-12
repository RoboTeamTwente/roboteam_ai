#include "ros/ros.h"
#include "DangerFinder/DangerFinder.h"
#include "io/IOManager.h"
#include "treeinterp/TreeInterpreter.h"
#include "utilities/RefStateManager.hpp"

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;

roboteam_msgs::World worldMsg;

int main(int argc, char* argv[]) {
    // Init ROS node
    ros::init(argc, argv, "StrategyNode");

    // init IOManager and subscribe to all topics immediately
    io::IOManager IOManager(true);
    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;

    bt::BehaviorTree strategy;

    // start looping
    // set the framerate to 50 Hz
    ros::Rate rate(50);

    auto factory = BTFactory::getFactory();

    factory.init();

    while (ros::ok()) {
        ros::spinOnce();

        // make ROS worldstate and geometry data globally accessible
        worldMsg = IOManager.getWorldState();
        geometryMsg = IOManager.getGeometryData();
        ai::World::set_world(worldMsg);
        ai::Field::set_field(geometryMsg.field);

        if (ai::World::didReceiveFirstWorld) {

            strategy = factory.getTree("DemoStrategy");
            bt::Node::Status status = strategy.Tick();

            if (status != bt::Node::Status::Running) {
                auto statusStr = bt::statusToString(status);
                // return failure, success or invalid
                ROS_DEBUG_STREAM_NAMED("Roboteam_ai", "Strategy result: " << statusStr.c_str() << "Shutting down...\n");
                break;
            }
        }
        rate.sleep();
    }

    // Terminate if needed
    if (strategy.getStatus() == bt::Node::Status::Running) {
        strategy.Terminate(bt::Node::Status::Running);
    }

    return 0;
}
