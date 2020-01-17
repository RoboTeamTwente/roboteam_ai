//
// Created by rolf on 16-01-20.
//
#include <gtest/gtest.h>
#include <control/BBTrajectories/BBTrajectory1D.h>

TEST(BBTrajectories, Onedimtest){
    float sum=0;
    for (int i = 0; i < 10000000; ++i) {
        BBTrajectory1D<float> test(i/100.0,1.0,0.1,1.0,2.0);
        sum+=test.getTotalTime();
    }
    std::cout<<sum<<std::endl;
}