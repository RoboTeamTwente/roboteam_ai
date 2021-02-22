//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>

#include "interface/widgets/mainWindow.h"
#include "AI.h"
#include "utilities/IOManager.h"

namespace rtt {

class ApplicationManager {

   private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);

    void runOneLoopCycle();
    bool fieldInitialized = false;
    bool robotsInitialized = false;
    std::unique_ptr<AI> ai;
    std::unique_ptr<rtt::ai::io::IOManager> io;


   public:
    void start(int ai_id);
    ApplicationManager() = default;
    ApplicationManager(ApplicationManager const&) = delete;
    ApplicationManager& operator=(ApplicationManager const&) = delete;
};

}  // namespace rtt

#endif  // ROBOTEAM_AI_APPLICATIONMANAGER_H
