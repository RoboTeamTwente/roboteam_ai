//
// Created by rolf on 25-10-18.
//
#include "../src/utilities/StrategyMapper.hpp"
#include "gtest/gtest.h"

//TODO: write a test that compiles all the "actual" trees once we have new tactics. For now, this should run but give a LOT of ROS warnings.
TEST(StrategyMapperTest, simpletest){
    BTFactory factory=BTFactory::getFactory();
    factory.init();
    // This should give 20 warning messages along the lines of Could not find the project: ... in the treeRepo. Using default strategy.
    rtt::ai::StrategyMapper::getMainStrategy();
}
