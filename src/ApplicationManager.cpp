//
// Created by mrlukasbos on 14-1-19.
//

#include "include/roboteam_ai/demo/JoystickDemo.h"
#include "include/roboteam_ai/coach/defence/DefenceDealer.h"
#include "include/roboteam_ai/coach/OffensiveCoach.h"
#include "include/roboteam_ai/coach/PassCoach.h"
#include "include/roboteam_ai/ApplicationManager.h"
#include <sstream>
#include "include/roboteam_ai/analysis/GameAnalyzer.h"
#include "include/roboteam_ai/interface/api/Output.h"
#include "include/roboteam_ai/coach/GetBallCoach.h"
#include "include/roboteam_ai/utilities/GameStateManager.hpp"
#include "include/roboteam_ai/interface/api/Input.h"
#include "include/roboteam_ai/interface/api/Toggles.h"
#include "include/roboteam_ai/utilities/Constants.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true, false);
    ai::GameStateManager::forceNewGameState(RefCommand::HALT);
}

void ApplicationManager::loop() {
  //   ros::Rate rate(ai::Constants::TICK_RATE());

   // BTFactory::makeTrees();
    double longestTick = 0.0;
    double timeTaken;
    int nTicksTaken = 0;
    double timeTakenOverNTicks = 0.0;


    while (true) {
      //  ros::Time begin = ros::Time::now();

        this->runOneLoopCycle();
       // rate.sleep();

       // ros::Time end = ros::Time::now();
        //timeTaken = (end - begin).toNSec() * 0.000001; // (ms)
        timeTakenOverNTicks += timeTaken;
        if (timeTaken > longestTick) {
            longestTick = timeTaken;
        }
        if (ai::interface::Output::showDebugTickTimeTaken() && ++nTicksTaken >= ai::Constants::TICK_RATE()) {
            std::stringstream ss;
            ss << "FrameRate: " << ai::Constants::TICK_RATE()*1000/timeTakenOverNTicks << " FPS | average: " << timeTakenOverNTicks / nTicksTaken << " ms/tick | longest: " << longestTick << " ms";
            if (nTicksTaken * longestTick < 2000 && timeTakenOverNTicks < 1200)
                std::cout << ss.str() << std::endl;
            else
                std::cerr << ss.str() << std::endl;
            nTicksTaken = 0;
            timeTakenOverNTicks = 0.0;
            longestTick = 0.0;
        }


        if (ai::robotDealer::RobotDealer::hasFree()) {
            if (ticksFree++ > 10) {
                ai::robotDealer::RobotDealer::refresh();
           //     BTFactory::makeTrees();
            }
        }
        else {
            ticksFree = 0;
        }
    }
}

void ApplicationManager::runOneLoopCycle() {
    if (weHaveRobots) {
        ai::analysis::GameAnalyzer::getInstance().start();

        // Will do things if this is a demo
        // otherwise wastes like 0.1 ms
        auto demoMsg = IOManager->getDemoInfo();
        demo::JoystickDemo::demoLoop(demoMsg);


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

        keeperTree = BTFactory::getKeeperTree();
        if (keeperTree && rtt::ai::robotDealer::RobotDealer::keeperExistsInWorld()) {
            keeperTree->tick();
        }


//        ros::Time begin;
//        ros::Time end;
//        if (ai::interface::Output::showCoachTimeTaken()) {
//            begin = ros::Time::now();
//        }

        rtt::ai::coach::getBallCoach->update();
        rtt::ai::coach::g_DefenceDealer.updateDefenderLocations();
        rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
        rtt::ai::coach::g_pass.updatePassProgression();

//        if (ai::interface::Output::showCoachTimeTaken()) {
//            end = ros::Time::now();
//            double timeTaken = (end - begin).toNSec() * 0.000001; // (ms)
//            std::cout << "The coaches are using " << timeTaken << " ms!" << std::endl;
//        }

        if (BTFactory::getCurrentTree() == "NaN") {
            std::cout << "NaN tree probably Halting" << std::endl;
            return;
        }

        strategy = BTFactory::getTree(BTFactory::getCurrentTree());
        Status status = strategy->tick();
        this->notifyTreeStatus(status);
    }
    else {
        std::cout <<"NO FIRST WORLD" << std::endl;
    //    ros::Duration(0.2).sleep();
        std::this_thread::sleep_for(std::chrono::microseconds(100000));

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

void ApplicationManager::notifyTreeStatus(bt::Node::Status status) {
    switch (status) {
    case Status::Running:break;
    case Status::Success:
        std::cout << " === TREE SUCCESS -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
        break;
    case Status::Failure:
        std::cout << " === TREE FAILURE -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
      break;
    case Status::Waiting:
        std::cout << " === Status returned: Waiting === " << std::endl;
        break;
    }
}

} // rtt
