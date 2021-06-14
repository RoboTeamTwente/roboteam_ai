//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>

#include "interface/widgets/mainWindow.h"
#include "stp/PlayChecker.hpp"
#include "stp/PlayDecider.hpp"
#include "stp/PlayEvaluator.h"

namespace rtt {

class ApplicationManager {
   public:
    explicit ApplicationManager(ai::interface::MainWindow* mainWindow);

   private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);

    void runOneLoopCycle();
    bool fieldInitialized = false;
    bool robotsInitialized = false;
    ai::interface::MainWindow* mainWindow;

    /**
     * Current best play as picked by checker + decider
     */
    ai::stp::Play* currentPlay{nullptr};

    /**
     * Checks which plays are valid out of all the plays
     */

    rtt::ai::stp::PlayChecker playChecker;

    /**
     * Checks, out of the valid plays, which play is the best to choose
     */
    rtt::ai::stp::PlayDecider playDecider;

    /**
     * Holds values for computed scores, gets reset every tick
     */
    rtt::ai::stp::PlayEvaluator playEvaluator;

    /**
     * Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     * @param field the current field state
     */
    void decidePlay(world::World* _world);

   public:
    void start();

    /**
     * The vector that contains all plays
     */
    static inline std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;

    ApplicationManager(ApplicationManager const&) = delete;
    ApplicationManager& operator=(ApplicationManager const&) = delete;
};

}  // namespace rtt

#endif  // ROBOTEAM_AI_APPLICATIONMANAGER_H
