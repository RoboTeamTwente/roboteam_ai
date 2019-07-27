//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/coach/PassCoach.h>
#include "io/IOManager.h"
#include "treeinterp/BTFactory.h"
#include "ros/ros.h"
#include <roboteam_ai/src/utilities/StrategyManager.h>

namespace rtt {

class ApplicationManager {
        FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);

    private:
        rtt::ai::io::IOManager* IOManager;

        int ticksFree = 0;
        bt::BehaviorTree::Ptr strategy;
        bt::BehaviorTree::Ptr keeperTree;

        void updateGameAnalyzer();
        void updateCoaches();
        void updateDemo();
        void updateStrategyChange();

        void runKeeper();
        void runStrategy();

        void notifyTreeStatus(bt::Node::Status status);

        void checkForFreeRobots();
        void runOneLoopCycle();
        bool weHaveRobots = false;

        ai::StrategyManager strategyManager;
        std::string oldKeeperTreeName = "";
        std::string oldStrategyName = "";

        // timing
        void updateTimer(int tickRate);
        double longestTick = 0.0;
        int nTicksTaken = 0;
        double timeTakenOverNTicks = 0.0;
        ros::Time begin = ros::Time::now();
        ros::Time end = ros::Time::now();

    public:
        void setup();
        void loop();
        void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
