#include "ApplicationManager.h"

#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>

#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "control/ControlModule.h"


namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void ApplicationManager::start(int id) {
    io = std::make_unique<io::IOManager>();
  rtt::ai::Constants::init();
  RTT_INFO("This AI is initialized with id ", id)
  // some default settings for different team ids (saves time while testing)
  if (id == 1) {
    // standard blue team on right
    rtt::SETTINGS.init(id);
    rtt::SETTINGS.setYellow(false);
    rtt::SETTINGS.setLeft(false);
    RTT_INFO("Initially playing as the BLUE team")
    RTT_INFO("We are playing on the RIGHT side of the field")
  } else {
    // standard yellow team on left
    rtt::SETTINGS.init(id);
    rtt::SETTINGS.setYellow(true);
    rtt::SETTINGS.setLeft(true);
    RTT_INFO("Initially playing as the YELLOW team")
    RTT_INFO("We are playing on the LEFT side of the field")
  }

  rtt::SETTINGS.setSerialMode(false);
  rtt::SETTINGS.setVisionIp("127.0.0.1");
  rtt::SETTINGS.setVisionPort(10006);
  rtt::SETTINGS.setRefereeIp("224.5.23.1");
  rtt::SETTINGS.setRefereePort(10003);
  rtt::SETTINGS.setRobothubSendIp("127.0.0.1");
  rtt::SETTINGS.setRobothubSendPort(20011);
  io->init(rtt::SETTINGS.getId());

  // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
    RTT_INFO("Start looping")
    RTT_INFO("Waiting for field_data and robots...")
    ai = std::make_unique<AI>();

    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            auto start = std::clock();
            runOneLoopCycle();

            RTT_WARNING("Time: ", (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000), " ms")
            RTT_WARNING("Time allowed: 16 ms")

            // publish settings, but limit this function call to only run 1 times/s at most
            t.limit([&]() { io->publishSettings(SETTINGS.toMessage()); }, 1);
        },
        ai::Constants::TICK_RATE());
}

/// Run everything with regard to behaviour trees
void ApplicationManager::runOneLoopCycle() {
    auto state = io->getState();
    if (state.has_field()) {
        if (!fieldInitialized) RTT_SUCCESS("Received first field message!")
        fieldInitialized = true;

        //Note these calls Assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
        auto worldMessage = state.last_seen_world();
        auto fieldMessage = state.field().field();
        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&worldMessage);
        }
        auto const &[_, world] = world::World::instance();
        world->updateWorld(worldMessage);

        if (!world->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting STP!")
            }
            robotsInitialized = true;


            world->updateField(fieldMessage);
            world->updatePositionControl();
            //world->updateFeedback(feedbackMap);

            ai->decidePlay(world);

        } else {
            if (robotsInitialized) {
                RTT_WARNING("No robots found in world. STP is not running")
                robotsInitialized = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        if (fieldInitialized) {
            RTT_WARNING("No field data present!")
            fieldInitialized = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    //TODO: move to AI
    rtt::ai::control::ControlModule::sendAllCommands(io);
    io->handleCentralServerConnection();
}



ApplicationManager::ApplicationManager(ai::interface::MainWindow *mainWindow) { this->mainWindow = mainWindow; }
}  // namespace rtt
