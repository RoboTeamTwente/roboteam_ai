#include "ros/ros.h"
#include "dangerfinder/DangerFinder.h"
#include "io/IOManager.h"
#include "utilities/Referee.hpp"
#include "utilities/StrategyManager.h"
#include "treeinterp/BTFactory.h"
#include "interface/Interface.h"
#include "interface/mainWindow.h"
#include "roboteam_ai/src/interface/widget.h"

#include <QApplication>

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;
namespace ui = rtt::ai::interface;

using Status = bt::Node::Status;
ui::Widget * widget;

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
    std::string currentTree = "victoryDanceStrategy";

    ros::Rate rate(50);

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

        if (!ai::World::didReceiveFirstWorld) continue;

        if (df::DangerFinder::instance().hasCalculated()) {
            df::DangerData dangerData = df::DangerFinder::instance().getMostRecentData();
        }

        // for refereedata:
        if (! ai::World::didReceiveFirstWorld) {
            ROS_ERROR("No first world");
            ros::Duration(0.2).sleep();
            continue;
        }

        // for referee_data:
        // ai::StrategyManager strategyManager;
        // std::string strategyName = strategyManager.getCurrentStrategyName();
        // strategy = factory.getTree(strategyName);

        strategy = factory.getTree(currentTree);

        Status status = strategy->tick();

        switch (status) {

        case Status::Running:
            break;

        case Status::Success:
            ROS_INFO_STREAM("Status returned: Success");
            ROS_INFO_STREAM(" === TREE CHANGE === ");
            currentTree = "ParallelSequenceStrategy";
            break;

        case Status::Failure:
            ROS_INFO_STREAM("Status returned: Failure");
            break;

        case Status::Invalid:
            ROS_INFO_STREAM("Status returned: Invalid");
            break;

        }

        if (widget) widget->update();
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

    // start the ros loop in seperate thread
    std::thread behaviourTreeThread = std::thread(&runBehaviourTrees);

    // initialize the interface
    QApplication a(argc, argv);
    ui::MainWindow w;

    ui::Widget wi;
    widget = &wi;

    // http://doc.qt.io/qt-5/signalsandslots.html
    // Here we connect the callback function of the mainwindow (which happens i.e. on button click)
    // to a 'listener' function from a different widget
    // it's like the listener function is called immediately.
    // This feels like black magic but it's actually quite pretty.
    QObject::connect(&w, &ui::MainWindow::rolescheckboxClicked, widget, &ui::Widget::setShowRoles);
    QObject::connect(&w, &ui::MainWindow::toggleTacticsCheckboxClicked, widget, &ui::Widget::setShowTactics);

    wi.show();
    w.show();
  return a.exec();
}

