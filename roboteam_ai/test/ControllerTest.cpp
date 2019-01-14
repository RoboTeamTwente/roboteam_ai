//
// Created by mrlukasbos on 11-1-19.
//
#include <gtest/gtest.h>
#include "../src/control/Controller.h"

namespace rtt {
namespace ai {
namespace control {


TEST(ControllerTest, it_calculates_proper_pid) {
    Controller c;
    ASSERT_EQ(c.controlPID(100), 0);

    c = Controller(10, 20, 30);
    double expectedP = 120;
    ASSERT_EQ(c.controlP(12), expectedP);
    double expectedI = (1.0/rtt::ai::constants::tickRate) * 12 * 20;
    ASSERT_EQ(c.controlI(12), expectedI);
    double expectedD = 30 * (12 / (1.0/rtt::ai::constants::tickRate));
    ASSERT_EQ(c.controlD(12), expectedD);

    // clear controller otherwise timeDiff makes the values different
    c = Controller(10, 20, 30);
    ASSERT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);

    // check if the setters work
    c = Controller(0, 0, 0);
    c.setP(10);
    c.setI(20);
    c.setD(30);
    ASSERT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);
}

TEST(ControllerTest, it_calculates_proper_pir) {
    Controller c;
    ASSERT_EQ(c.controlPIR(100, 100), 0);

    c = Controller(10, 20, 30);
    double expectedP = 120;
    ASSERT_EQ(c.controlP(12), expectedP);
    double expectedI = (1.0/rtt::ai::constants::tickRate) * 12 * 20;
    ASSERT_EQ(c.controlI(12), expectedI);
    double expectedR = 30 * 12 * -1;
    ASSERT_EQ(c.controlR(12), expectedR);


    // clear controller otherwise timeDiff makes the values different
    c = Controller(10, 20, 30);
    ASSERT_EQ(c.controlPIR(12, 12), expectedP + expectedI + expectedR);
}


} // control
} // ai
} // rtt