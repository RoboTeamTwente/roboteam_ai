#include "ApplicationManager.h"

#include <roboteam_ai/src/demo/JoystickDemo.h>
#include <roboteam_ai/src/coach/defence/DefenceDealer.h>
#include <sstream>
#include "roboteam_ai/src/analysis/GameAnalyzer.h"
#include "roboteam_ai/src/interface/api/Output.h"
#include "roboteam_ai/src/coach/GetBallCoach.h"
#include "roboteam_ai/src/utilities/GameStateManager.hpp"
#include "roboteam_ai/src/interface/api/Input.h"
#include "roboteam_ai/src/interface/api/Toggles.h"
#include "roboteam_ai/src/utilities/Constants.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true, false);
    ai::GameStateManager::forceNewGameState(RefCommand::HALT);
}

void ApplicationManager::loop() {
    int tickRate = ai::Constants::TICK_RATE();
    ros::Rate rate(tickRate);

    while (ros::ok()) {
        updateTimer(tickRate);
        runOneLoopCycle();
        checkForFreeRobots();

        rate.sleep();
    }
}

void ApplicationManager::checkForFreeRobots() {
    // fix free bug (hack?)
    if (ai::robotDealer::RobotDealer::hasFree()) {
        if (ticksFree++ > 10) {
            ai::robotDealer::RobotDealer::refresh();
        }
    }
    else {
        ticksFree = 0;
    }
}

void ApplicationManager::runOneLoopCycle() {
    if (weHaveRobots) {
        updateGameAnalyzer();
        updateCoaches();
        updateDemo();
        updateStrategyChange();

        runKeeper();
        runStrategy();
    }
    else {
        std::cout <<"WE DO NOT HAVE ROBOTS" << std::endl;
        ros::Duration(0.2).sleep();
    }

    weHaveRobots = ai::world::world->weHaveRobots();
}

void ApplicationManager::checkForShutdown() {
    // Terminate if needed
    if (strategy->getStatus() == Status::Running) {
        strategy->terminate(Status::Running);
    }
    ai::analysis::GameAnalyzer::getInstance().stop();
}

void ApplicationManager::updateGameAnalyzer() {
    ai::analysis::GameAnalyzer::getInstance().start();
}

void ApplicationManager::updateCoaches() {
    rtt::ai::coach::getBallCoach->update();
    rtt::ai::coach::g_DefenceDealer.updateDefenderLocations();
    rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
    rtt::ai::coach::g_pass.updatePassProgression();
}

void ApplicationManager::updateDemo() {
    // Will do things if this is a demo
    auto demoMsg = IOManager->getDemoInfo();
    demo::JoystickDemo::demoLoop(demoMsg);
}

void ApplicationManager::updateStrategyChange() {
    auto gameState = ai::GameStateManager::getCurrentGameState();

    std::string strategyName = gameState.strategyName;
    std::string keeperTreeName = gameState.keeperStrategyName;

    bool strategyChanged = oldStrategyName != strategyName;
    bool keeperStrategyChanged = oldKeeperTreeName != keeperTreeName;

    if (strategyChanged) {
        BTFactory::setCurrentTree(strategyName);
        oldStrategyName = strategyName;
    }

    if (keeperStrategyChanged) {
        BTFactory::setKeeperTree(keeperTreeName);
        oldKeeperTreeName = keeperTreeName;
    }

    if (keeperStrategyChanged || strategyChanged) {
        ai::robotDealer::RobotDealer::refresh();
    }

    rtt::ai::robotDealer::RobotDealer::setKeeperID(gameState.keeperId);
}

void ApplicationManager::runKeeper() {
    keeperTree = BTFactory::getKeeperTree();
    if (keeperTree && rtt::ai::robotDealer::RobotDealer::keeperExistsInWorld()) {
        keeperTree->tick();
    }
}

void ApplicationManager::runStrategy() {
    if (BTFactory::getCurrentTree() == "NaN") {
        std::cout << "NaN tree probably Halting" << std::endl;
        return;
    }

    strategy = BTFactory::getTree(BTFactory::getCurrentTree());
    Status status = strategy->tick();

    notifyTreeStatus(status);
}


void ApplicationManager::notifyTreeStatus(bt::Node::Status status) {
    switch (status) {
    case Status::Running:
        break;
    case Status::Success:
        std::cout << " === TREE SUCCESS -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
        break;
    case Status::Failure:
        std::cout << " === TREE FAILURE -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
        break;
    case Status::Waiting:
        ROS_INFO_STREAM("Status returned: Waiting");
        break;
    }
}

void ApplicationManager::updateTimer(int tickRate) {
    end = ros::Time::now();

    double timeTaken = (end - begin).toNSec() * 0.000001; // (ms)
    timeTakenOverNTicks += timeTaken;
    if (timeTaken > longestTick) {
        longestTick = timeTaken;
    }

    if (ai::interface::Output::showDebugTickTimeTaken() && ++nTicksTaken >= tickRate) {
        std::stringstream ss;
        ss <<
           "FrameRate: " << 1.0 * (int)( 1.0  * tickRate*1000.0/timeTakenOverNTicks ) << " FPS | "     <<
           "Average: "   << 0.1 * (int)( 10.0 * timeTakenOverNTicks / nTicksTaken   ) << " ms/tick | " <<
           "Longest: "   << 0.1 * (int)( 10.0 * longestTick                         ) << " ms" ;
        if (nTicksTaken * longestTick < 2000.0 && timeTakenOverNTicks < 1200.0) {
            std::cout << ss.str() << std::endl;
        }
        else {
            std::cerr << ss.str() << std::endl;
        }

        nTicksTaken = 0;
        timeTakenOverNTicks = 0.0;
        longestTick = 0.0;
    }

    begin = end;
}

} // rtt
