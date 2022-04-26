#pragma once
#include <gtest/gtest_prod.h>

#include "interface/widgets/mainWindow.h"
#include <interface/InterfaceControllerServer.h>

#include <stp/Play.hpp>

namespace rtt {

class STPManager {
   public:
    explicit STPManager(ai::interface::MainWindow* mainWindow);

   private:
    FRIEND_TEST(STPManagerTest, it_handles_ROS_data);

    void runOneLoopCycle();
    bool fieldInitialized = false;
    bool robotsInitialized = false;
    ai::interface::MainWindow* mainWindow;

    /**
     * Current best play as picked by the playDecider
     */
    ai::stp::Play* currentPlay{nullptr};

    /**
     * Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     */
    void decidePlay(world::World* _world);
    bool needsNewPlay();

   public:
    void start();

    /**
     * The vector that contains all plays
     */
    static inline std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;
    Interface::InterfaceControllerServer interfaceController;

    STPManager(STPManager const&) = delete;
    STPManager& operator=(STPManager const&) = delete;
};

}  // namespace rtt