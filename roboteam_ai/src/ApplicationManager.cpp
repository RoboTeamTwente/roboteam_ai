//
// Created by mrlukasbos on 14-1-19.
//

#include <roboteam_ai/src/demo/JoystickDemo.h>
#include "ApplicationManager.h"
#include "utilities/Referee.hpp"
#include "utilities/StrategyManager.h"
#include "utilities/Field.h"
#include <sstream>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true);
    factory = BTFactory::getFactory();
    factory.init();
    BTFactory::setCurrentTree("haltStrategy");
    BTFactory::setKeeperTree("keeperTest1");
}

void ApplicationManager::loop() {
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
    this->updateROSData();

    if (ai::World::didReceiveFirstWorld) {
        if (BTFactory::getCurrentTree() == "NaN") {
            ROS_INFO("NaN tree probably Halting");
            return;
        }

        ai::analysis::GameAnalyzer::getInstance().start();

        // Will do things if this is a demo
        // otherwise wastes like 0.1 ms
        auto demomsg = IOManager->getDemoInfo();
        demo::JoystickDemo::demoLoop(demomsg);

        if (ai::interface::InterfaceValues::usesRefereeCommands()) {
            this->handleRefData();
        }
        // TODO: change this later so the referee tells you this
        // TODO enable for keeper
//        robotDealer::RobotDealer::setKeeperID(0);
//        keeperTree = BTFactory::getKeeperTree();
//        Status keeperStatus = keeperTree->tick();

        strategy = factory.getTree(BTFactory::getCurrentTree());
        Status status = strategy->tick();
        this->notifyTreeStatus(status);

        rtt::ai::coach::g_offensiveCoach.calculateNewPositions();
    }
    else {
        ROS_ERROR("No first world");
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

void ApplicationManager::updateROSData() {
    // make ROS world_state and geometry data globally accessible
    worldMsg = IOManager->getWorldState();
    geometryMsg = IOManager->getGeometryData();
    refereeMsg = IOManager->getRefereeData();

    ai::World::set_world(worldMsg);
    ai::Field::set_field(geometryMsg.field);
    ai::Referee::setRefereeData(refereeMsg);
}

void ApplicationManager::handleRefData() {
    ai::StrategyManager strategyManager;
    // Warning, this means that the names in strategy manager needs to match one on one with the JSON names
    // might want to build something that verifies this
    auto oldStrategy = BTFactory::getCurrentTree();
    std::string strategyName = strategyManager.getCurrentStrategyName(refereeMsg.command);
    if (oldStrategy != strategyName) {
        BTFactory::getFactory().init();
    }
    BTFactory::setCurrentTree(strategyName);
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
