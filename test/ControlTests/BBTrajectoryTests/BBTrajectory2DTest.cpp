//
// Created by rolf on 17-1-20.
//

#include <gtest/gtest.h>
#include "control/BBTrajectories/BBTrajectory2D.h"
#include "control/BBTrajectories/BBTrajectory2DAsync.h"
#include "test/helpers/WorldHelper.h"
#include <roboteam_utils/Timer.h>
using namespace rtt;

void print(BBTrajectory2D<float> &traj){
    float maxTime=fmax(traj.x.getTotalTime(),traj.y.getTotalTime());
    float intervals = 30;
    std::cout<<"___________"<<std::endl;
    for (int i = 0; i <= intervals; ++ i) {
        std::cout<<traj.x.getValues(maxTime/intervals*i).pos<<" ";
        std::cout<<traj.y.getValues(maxTime/intervals*i).pos<<" ";
        std::cout<<maxTime/intervals*i<<";"<<std::endl;
    }
}
void print(BBTrajectory2DAsync<float> &traj){
    float maxTime=fmax(traj.x.getTotalTime(),traj.y.getTotalTime());
    float intervals = 30;
    std::cout<<"___________"<<std::endl;
    for (int i = 0; i <= intervals; ++ i) {
        std::cout<<traj.x.getValues(maxTime/intervals*i).pos<<" ";
        std::cout<<traj.y.getValues(maxTime/intervals*i).pos<<" ";
        std::cout<<maxTime/intervals*i<<";"<<std::endl;
    }
}
TEST(BBTrajectories, BBTrajectory2D) {

    BBTrajectory2D<float> trajectory;
    proto::GeometryFieldSize field;
    field.set_field_width(9.0);
    field.set_field_length(12.0);
    std::chrono::nanoseconds totalTime = std::chrono::nanoseconds(0);
    for (int j = 0; j < 100; ++ j) {
        Vector2 startPos = testhelpers::WorldHelper::getRandomFieldPosition(field);
        Vector2 initialVel = testhelpers::WorldHelper::getRandomVelocity();
        Vector2 endPos = testhelpers::WorldHelper::getRandomFieldPosition(field);
        float maxVel = testhelpers::WorldHelper::getRandomValue(1, 8.0);
        float maxAcc = testhelpers::WorldHelper::getRandomValue(1, 4.0);

        std::chrono::nanoseconds start = std::chrono::system_clock::now().time_since_epoch();
        trajectory.generateSyncedTrajectory(startPos, initialVel, endPos, maxVel, maxAcc);
        std::chrono::nanoseconds end = std::chrono::system_clock::now().time_since_epoch();
        float bestTime=fmax(trajectory.x.getTotalTime(),trajectory.y.getTotalTime());
        float angle=0;

        for (int i = 0; i < 100; ++ i) {
            angle+=M_PI_2/100.0;
            BBTrajectory2D<float> testTwo(startPos,initialVel,endPos,maxVel,maxAcc,angle);
            float xTime=testTwo.x.getTotalTime();
            float yTime=testTwo.y.getTotalTime();
            float computedTime=fmax(xTime,yTime);
            if (bestTime>computedTime){
                print(trajectory);
                print(testTwo);
                std::cout<<"params"<<startPos<<" "<<initialVel<<" "<<endPos<<" "<<maxVel<<" "<<maxAcc<<" "<<angle<<std::endl;
            }
            EXPECT_LE(bestTime,fmax(xTime,yTime));
        }
        totalTime += (end - start);
    }
    std::cout << totalTime.count()/100.0 << std::endl;
}
TEST(BBTrajectories, BBTrajectory2D2) {

    BBTrajectory2D<float> trajectory;
    proto::GeometryFieldSize field;
    field.set_field_width(9.0);
    field.set_field_length(12.0);
    std::chrono::nanoseconds totalTime = std::chrono::nanoseconds(0);
    for (int j = 0; j < 6000; ++ j) {
        Vector2 startPos = testhelpers::WorldHelper::getRandomFieldPosition(field);
        Vector2 initialVel = testhelpers::WorldHelper::getRandomVelocity();
        Vector2 endPos = testhelpers::WorldHelper::getRandomFieldPosition(field);
        float maxVel = testhelpers::WorldHelper::getRandomValue(0, 8.0);
        float maxAcc = testhelpers::WorldHelper::getRandomValue(0, 4.0);
        float alpha = testhelpers::WorldHelper::getRandomValue(0, M_PI_2);
        std::chrono::nanoseconds start = std::chrono::system_clock::now().time_since_epoch();
        trajectory.generateTrajectory(startPos, initialVel, endPos, maxVel, maxAcc, alpha);
        std::chrono::nanoseconds end = std::chrono::system_clock::now().time_since_epoch();
        totalTime+=(end-start);
    }
    std::cout << totalTime.count()/6000.0 << std::endl;
}
TEST(BBTrajectories, BBTrajectory2DAsync){
    BBTrajectory2DAsync<float> trajectory2DAsync;
    Vector2 initialPos(-5,1);
    Vector2 initialVel(-1,1);
    Vector2 endPos(-4,0);
    Vector2 lineDir(-1,0);
    float maxVel=2.0;
    float maxAcc=1.5;
    trajectory2DAsync.generateAsyncTrajectory(initialPos,initialVel,endPos,maxVel,maxAcc,lineDir);
    print(trajectory2DAsync);

    BBTrajectory2D<float> trajectory2D(initialPos,initialVel,endPos,maxVel,maxAcc);
    print(trajectory2D);

}