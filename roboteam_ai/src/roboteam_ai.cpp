#include "ros/ros.h"
#include "dangerfinder/DangerFinder.h"
#include "io/IOManager.h"
#include "utilities/Referee.hpp"
#include "utilities/StrategyManager.h"
#include "treeinterp/BTFactory.h"
#include "interface/mainWindow.h"
#include <QApplication>

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;
namespace ui = rtt::ai::interface;

using Status = bt::Node::Status;

std::shared_ptr<ui::MainWindow> window;

void runBehaviourTrees() {
    // init IOManager and subscribe to all topics immediately
    io::IOManager IOManager(true);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;
    roboteam_msgs::RefereeData refereeMsg;

    bt::BehaviorTree::Ptr strategy;

    // Where we keep our trees
    auto factory = BTFactory::getFactory();
    factory.init();

    // Start running this tree first
    ros::Rate rate(rtt::ai::constants::tickRate); //50 Hz

    BTFactory::setCurrentTree("SimpleStrategy");

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();

        // make ROS world_state and geometry data globally accessible
        worldMsg = IOManager.getWorldState();
        geometryMsg = IOManager.getGeometryData();
        refereeMsg = IOManager.getRefereeData();
        ai::World::set_world(worldMsg);
        ai::Field::set_field(geometryMsg.field);
        ai::Referee::setRefereeData(refereeMsg);

        if (df::DangerFinder::instance().hasCalculated()) {
            df::DangerData dangerData = df::DangerFinder::instance().getMostRecentData();
        }

        // for referee_data:
        if (! ai::World::didReceiveFirstWorld) {
            ROS_ERROR("No first world");
            ros::Duration(0.2).sleep();
            continue;
        }

        // for referee_data:
        // ai::StrategyManager strategyManager;
        // std::string strategyName = strategyManager.getCurrentStrategyName();
        // strategy = factory.getTree(strategyName);

        strategy = factory.getTree(BTFactory::getCurrentTree());

//
//        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        Status status = strategy->tick();
//        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//
//        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//        std::cout << "Tick took:  " << time_span.count()*1000 << " ms." << std::endl;

        switch (status) {
            case Status::Running:
                break;
            case Status::Success:
                ROS_INFO_STREAM("Status returned: Success");
                ROS_INFO_STREAM(" === TREE CHANGE === ");

                break;

            case Status::Failure:
                ROS_INFO_STREAM("Status returned: Failure");
                break;
            case Status::Waiting:
                ROS_INFO_STREAM("Status returned: Waiting");
                break;
        }

        rate.sleep();
    }

    // Terminate if needed
    if (strategy->getStatus() == Status::Running) {
        strategy->terminate(Status::Running);
    }
}

int main(int argc, char* argv[]) {
    // Init ROS node in main thread
    ros::init(argc, argv, "StrategyNode");

    // start the ros loop in separate thread
    std::thread behaviourTreeThread = std::thread(&runBehaviourTrees);

    // initialize the interface
    QApplication a(argc, argv);
    window = std::make_shared<ui::MainWindow>();
    window->show();

    return a.exec();
}

