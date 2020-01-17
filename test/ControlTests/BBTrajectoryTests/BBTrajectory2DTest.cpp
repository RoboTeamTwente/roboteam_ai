//
// Created by rolf on 17-1-20.
//

#include <gtest/gtest.h>
#include "control/BBTrajectories/BBTrajectory2D.h"
#include "test/helpers/WorldHelper.h"
#include <roboteam_utils/Timer.h>
using namespace rtt;
TEST(BBTrajectories, BBTrajectory2D) {

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
        std::chrono::nanoseconds start = std::chrono::system_clock::now().time_since_epoch();
        trajectory.generateSyncedTrajectory(startPos, initialVel, endPos, maxVel, maxAcc);
        std::chrono::nanoseconds end = std::chrono::system_clock::now().time_since_epoch();
        totalTime += (end - start);
    }
    std::cout << totalTime.count()/6000.0 << std::endl;
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
