#pragma once
#include <gtest/gtest_prod.h>

#include <interface/InterfaceControllerServer.h>

#include <stp/Play.hpp>

namespace rtt {

class STPManager {
   public:
    explicit STPManager();

   private:
    FRIEND_TEST(STPManagerTest, it_handles_ROS_data);

    void runOneLoopCycle();
    bool fieldInitialized = false;
    bool robotsInitialized = false;

    ai::stp::Play* currentPlay{nullptr};

    /**
     * Ensures the correct play is selected and provided with the current data
     * @param world The current world state
     */
    void updatePlay(world::World* world);

    /** Returns whether we need to pick a new play */
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