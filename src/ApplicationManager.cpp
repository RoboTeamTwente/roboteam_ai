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
#include <roboteam_utils/Timer.h>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

void ApplicationManager::start() {

    // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT);

    int fpsUpdateRate = 5;
    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop([&]() {
        this->runOneLoopCycle();
        amountOfCycles++;

        t.limit([&](){
          ai::interface::Input::setFps(amountOfCycles * fpsUpdateRate);
          amountOfCycles = 0;
        }, fpsUpdateRate);
    }, ai::Constants::TICK_RATE());
}
void ApplicationManager::checkForFreeRobots() {// robotdealer hack
    if (ai::robotDealer::RobotDealer::hasFree()) {
          if (ticksFree++ > 10) { ai::robotDealer::RobotDealer::refresh(); }
      }
      else { ticksFree = 0; }
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



          rtt::ai::coach::getBallCoach->update();
          rtt::ai::coach::g_DefenceDealer.updateDefenderLocations();
          rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
          rtt::ai::coach::g_pass.updatePassProgression();


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

    /*
    * This is a hack performed at the robocup.
    * It does a soft refresh when robots are not properly claimed by robotdealer.
    */
    checkForFreeRobots();
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
