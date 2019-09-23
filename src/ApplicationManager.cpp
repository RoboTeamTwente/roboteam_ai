//
// Created by mrlukasbos on 14-1-19.
//

#include "demo/JoystickDemo.h"
#include "coach/defence/DefenceDealer.h"
#include "coach/OffensiveCoach.h"
#include "coach/PassCoach.h"
#include "ApplicationManager.h"
#include <sstream>
#include <Settings/Settings.h>
#include "analysis/GameAnalyzer.h"
#include "interface/api/Output.h"
#include "coach/GetBallCoach.h"
#include "utilities/GameStateManager.hpp"
#include "interface/api/Input.h"
#include "interface/api/Toggles.h"
#include "utilities/Constants.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::setup() {
    ai::GameStateManager::forceNewGameState(RefCommand::HALT);
}

void ApplicationManager::loop() {

   // BTFactory::makeTrees();
    double longestTick = 0.0;
    double timeTaken;
    int nTicksTaken = 0;
    double timeTakenOverNTicks = 0.0;

    std::chrono::milliseconds now_time;
    std::chrono::milliseconds last_call_time = now_time;
    std::chrono::milliseconds last_fps_count_time = now_time;

    bool ok = true;

    int cyclesInThisSecond = 0;

    int lastFPS = 0;
    while(ok) {

        now_time = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
        );
        auto diff = now_time - last_call_time;
        auto timeDiff =std::chrono::milliseconds(16); //a little over 60hz
        if(diff > timeDiff) {
            this->runOneLoopCycle();

            std::chrono::milliseconds afterOneCycleTime = std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch());

            cyclesInThisSecond++;

            auto fps_diff = now_time - last_fps_count_time;
            auto fps_timestep =std::chrono::milliseconds(200); // one second
            if(fps_diff > fps_timestep) {

                std::cout << "FPS: " << cyclesInThisSecond << std::endl;
                lastFPS = cyclesInThisSecond;
                cyclesInThisSecond=0;
                last_fps_count_time = now_time;
            }
            ai::interface::Input::setFps(lastFPS*5);

            if (ai::robotDealer::RobotDealer::hasFree()) {
                if (ticksFree++ > 10) {
                    ai::robotDealer::RobotDealer::refresh();
                }
            }
            else {
                ticksFree = 0;
            }
            last_call_time = now_time;
        }else {
            std::this_thread::sleep_for((timeDiff-diff) * .9); // sleep for the diff - arbitrary margin
        }
    }

}

void ApplicationManager::runOneLoopCycle() {

    // publish settings every second
    if (publishSettingTicks > ai::Constants::TICK_RATE()) {
        io::io.publishSettings(SETTINGS.toMessage());
        publishSettingTicks = 0;
    } else {
        publishSettingTicks++;
    }

    if (weHaveRobots && io::io.hasReceivedGeom) {
        ai::analysis::GameAnalyzer::getInstance().start();

        // Will do things if this is a demo
        // otherwise wastes like 0.1 ms
        auto demoMsg = io::io.getDemoInfo();
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
