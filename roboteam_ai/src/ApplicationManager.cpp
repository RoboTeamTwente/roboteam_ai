//
// Created by mrlukasbos on 14-1-19.
//


#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/coach/defence/DefenceDealer.h>
#include "ApplicationManager.h"
#include <sstream>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true);

    BTFactory::setCurrentTree("QualificationStrategy");
    BTFactory::setKeeperTree("SingleKeeperTactic");
}

void ApplicationManager::loop() {
    std::cout << "loop" << std::endl;
    ros::Rate rate(ai::Constants::TICK_RATE());
    double longestTick = 0.0;
    double timeTaken;
    int nTicksTaken = 0;
    double timeTakenOverNTicks = 0.0;
    while (ros::ok()) {
        ros::Time begin = ros::Time::now();

        this->runOneLoopCycle();
        rate.sleep();

        ros::Time end = ros::Time::now();
        timeTaken = (end - begin).toNSec() * 0.000001; // (ms)
        timeTakenOverNTicks += timeTaken;
        if (timeTaken > longestTick) {
            longestTick = timeTaken;
        }
        if (ai::interface::InterfaceValues::showDebugTickTimeTaken() && ++nTicksTaken >= ai::Constants::TICK_RATE()) {
            std::stringstream ss;
            ss << "The last " << nTicksTaken << " ticks took " << timeTakenOverNTicks << " ms, which gives an average of " << timeTakenOverNTicks / nTicksTaken << " ms / tick. The longest tick took " << longestTick << " ms!";
            if (nTicksTaken * longestTick < 2000 && timeTakenOverNTicks < 1200)
                std::cout << ss.str() << std::endl;
            else
                std::cerr << ss.str() << std::endl;
            nTicksTaken = 0;
            timeTakenOverNTicks = 0.0;
            longestTick = 0.0;
        }
    }
}

void ApplicationManager::runOneLoopCycle() {
    ros::spinOnce();

    if (ai::world::world->weHaveRobots()) {

        if (BTFactory::getCurrentTree() == "NaN") {
            ROS_INFO("NaN tree probably Halting");
            return;
        }

        ai::analysis::GameAnalyzer::getInstance().start();

        // Will do things if this is a demo
        // otherwise wastes like 0.1 ms
        auto demomsg = IOManager->getDemoInfo();
        demo::JoystickDemo::demoLoop(demomsg);
        rtt::ai::robotDealer::RobotDealer::setUseSeparateKeeper(true);

        if (rtt::ai::robotDealer::RobotDealer::usesSeparateKeeper()) {

            if (ai::robotDealer::RobotDealer::getKeeperID() == -1) {
                std::cout << "setting keeper id" << std::endl;
                ai::robotDealer::RobotDealer::setKeeperID(ai::world::world->getUs().at(0).id);


            }
            keeperTree = BTFactory::getKeeperTree();
            Status keeperStatus = keeperTree->tick();
        }  else {
            BTFactory::makeTrees();

        }
        strategy = BTFactory::getTree(BTFactory::getCurrentTree());
        Status status = strategy->tick();
        this->notifyTreeStatus(status);

        rtt::ai::coach::g_DefenceDealer.setDoUpdate();
        rtt::ai::coach::g_offensiveCoach.calculateNewPositions();
    }
    else {
        std::cout <<"NO FIRST WORLD" << std::endl;
        ros::Duration(0.2).sleep();
    }
}

void ApplicationManager::checkForShutdown() {
    // Terminate if needed
    if (strategy->getStatus() == Status::Running) {
        strategy->terminate(Status::Running);
    }
    ai::analysis::GameAnalyzer::getInstance().stop();
}

void ApplicationManager::notifyTreeStatus(bt::Node::Status status) {
    switch (status) {
    case Status::Running:break;
    case Status::Success:ROS_INFO_STREAM("Status returned: Success");
        ROS_INFO_STREAM(" === TREE CHANGE === ");
            BTFactory::setCurrentTree("haltStrategy");
        break;
    case Status::Failure:ROS_INFO_STREAM("Status returned: Failure");
        break;
    case Status::Waiting:ROS_INFO_STREAM("Status returned: Waiting");
        break;
    }
}

} // rtt
