//
// Created by mrlukasbos on 14-1-19.
//

#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/coach/defence/DefenceDealer.h>
#include "ApplicationManager.h"
#include <sstream>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/coach/GetBallCoach.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true, false);

    BTFactory::setCurrentTree("halt_strategy");
    BTFactory::setKeeperTree("keeper_default_tactic");
    rtt::ai::robotDealer::RobotDealer::setUseSeparateKeeper(true);

}

void ApplicationManager::loop() {
    ros::Rate rate(ai::Constants::TICK_RATE());
    BTFactory::makeTrees();
    double longestTick = 0.0;
    double timeTaken;
    int nTicksTaken = 0;
    double timeTakenOverNTicks = 0.0;
    BTFactory::makeTrees();
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
        if (ai::interface::Output::showDebugTickTimeTaken() && ++nTicksTaken >= ai::Constants::TICK_RATE()) {
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


        auto gamestate = ai::GameStateManager::getCurrentGameState();
        std::string strategyName = gamestate.strategyName;
        std::string keeperTreeName = gamestate.keeperStrategyName;

        bool strategyChanged = oldStrategy != strategyName;
        bool keeperStrategyChanged = oldKeeperTreeName != keeperTreeName;

        if (strategyChanged) {
            BTFactory::setCurrentTree(strategyName);
            oldStrategy = strategyName;
        }

        if (keeperStrategyChanged) {
            std::cout << "changing keeper tree" << std::endl;
            BTFactory::setKeeperTree(keeperTreeName);
            oldKeeperTreeName = keeperTreeName;
        }

        if (keeperStrategyChanged || strategyChanged) {
            ai::robotDealer::RobotDealer::refresh();
        }



        ai::robotDealer::RobotDealer::setUseSeparateKeeper(gamestate.useKeeper);
        ai::robotDealer::RobotDealer::setKeeperID(gamestate.keeperId);




        if (rtt::ai::robotDealer::RobotDealer::usesSeparateKeeper()) {
            keeperTree = BTFactory::getKeeperTree();
            if (keeperTree && rtt::ai::robotDealer::RobotDealer::keeperExistsInWorld()) {
                keeperTree->tick();
            }
        }

        rtt::ai::coach::getBallCoach->update();
        rtt::ai::coach::g_DefenceDealer.updateDefenderLocations();
        rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
        rtt::ai::coach::g_pass.updatePassProgression();

        strategy = BTFactory::getTree(BTFactory::getCurrentTree());
        Status status = strategy->tick();
        this->notifyTreeStatus(status);

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
    case Status::Success:
        std::cout << " === TREE SUCCESS -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        BTFactory::setCurrentTree("normal_play_strategy");
        BTFactory::setKeeperTree("keeper_default_tactic");

        ai::robotDealer::RobotDealer::refresh();
        break;
    case Status::Failure:
        std::cout << " === TREE FAILURE -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        BTFactory::setCurrentTree("normal_play_strategy");
        BTFactory::setKeeperTree("keeper_default_tactic");

        ai::robotDealer::RobotDealer::refresh();
        break;
    case Status::Waiting:ROS_INFO_STREAM("Status returned: Waiting");
        break;
    }
}

} // rtt
