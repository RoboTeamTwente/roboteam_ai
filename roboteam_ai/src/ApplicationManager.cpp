//
// Created by mrlukasbos on 14-1-19.
//


#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/utilities/DefensiveCoach.h>
#include "ApplicationManager.h"
#include "utilities/Referee.hpp"
#include "utilities/StrategyManager.h"

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
    std::vector<double> totalTime;
    while (ros::ok()) {
        ros::Time begin = ros::Time::now();

        this->runOneLoopCycle();

        ros::Time end = ros::Time::now();
        if (ai::Constants::SHOW_LONGEST_TICK()) {

            timeTaken = (end - begin).toNSec() * 0.000001;

            totalTime.push_back(timeTaken);
            if (totalTime.size() > 100)
                totalTime.erase(totalTime.begin());

            if (timeTaken > longestTick) {
                if (timeTaken > 200) {
                    std::cout << "tick took longer than 200ms!!" << std::endl;
                }
                else {
                    longestTick = timeTaken;
                    int totalTicks = 0;
                    double total = 0.0;
                    for (auto &time : totalTime) {
                        totalTicks ++;
                        total += time;
                    }
                    total = total/totalTicks;

                    std::cout << "    average tick time (last 100 ticks) : " << total << " ms" << std::endl;
                    std::cout << "longest tick took: " << longestTick << " ms" << std::endl;

                }
            }
        }
        rate.sleep();
    }
}

void ApplicationManager::runOneLoopCycle() {
    ros::spinOnce();
    this->updateROSData();
    this->updateDangerfinder();
    std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> vis;
    auto locations=ai::coach::DefensiveCoach::decideDefenderLocations(3);
    for (auto location : locations){
        auto pair=make_pair(location,Qt::red);
        vis.emplace_back(pair);
    }
    ai::interface::Drawer::setTestLines(vis);
    std::vector<std::pair<Vector2,QColor>> vis2;
    auto locations2=ai::coach::DefensiveCoach::decideDefenderLocations2(3,0);
    for (auto location : locations2){
        auto pair=make_pair(location,Qt::green);
        vis2.emplace_back(pair);
    }
    ai::interface::Drawer::setTestPoints(vis2);
    if (ai::World::didReceiveFirstWorld) {
        if (BTFactory::getCurrentTree() == "NaN") {
            ROS_INFO("NaN tree probably Halting");
            return;
        }

        // Will do things if this is a demo
        // otherwise wastes like 0.1 ms
        auto demomsg = IOManager->getDemoInfo();
        demo::JoystickDemo::demoLoop(demomsg);

        if (ai::interface::InterfaceValues::usesRefereeCommands()) {
            this->handleRefData();
        }
        // TODO: change this later so the referee tells you this
        // TODO enable for keeper
        robotDealer::RobotDealer::setKeeperID(0);
        keeperTree = BTFactory::getKeeperTree();
        Status keeperStatus = keeperTree->tick();

        strategy = factory.getTree(BTFactory::getCurrentTree());
        Status status = strategy->tick();
        this->notifyTreeStatus(status);
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

void ApplicationManager::updateDangerfinder() {
    if (df::DangerFinder::instance().hasCalculated()) {
        dangerData = df::DangerFinder::instance().getMostRecentData();
    }
}

void ApplicationManager::handleRefData() {
    ai::StrategyManager strategyManager;
    std::string strategyName = strategyManager.getCurrentStrategyName(refereeMsg.command);
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
