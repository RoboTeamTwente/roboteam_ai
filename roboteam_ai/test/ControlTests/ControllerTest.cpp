//
// Created by mrlukasbos on 11-1-19.
//
#include <gtest/gtest.h>
#include "roboteam_ai/src/control/Controller.h"

namespace rtt {
namespace ai {
namespace control {

TEST(ControllerTest, it_calculates_proper_pid) {

    // Empty controller
    Controller c;
    EXPECT_EQ(c.controlPID(100), 0);

    // controller with P, I and D
    c = Controller(10, 20, 30);
    double expectedP = 120;
    EXPECT_EQ(c.controlP(12), expectedP);
    double expectedI = (1.0/rtt::ai::Constants::TICK_RATE()) * 12 * 20;
    EXPECT_EQ(c.controlI(12), expectedI);
    double expectedD = 30 * (12 / (1.0/rtt::ai::Constants::TICK_RATE()));
    EXPECT_EQ(c.controlD(12), expectedD);

    // clear controller otherwise timeDiff makes the values different
    c = Controller(10, 20, 30);
    EXPECT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);

    // check if the setters work
    c = Controller(0, 0, 0);
    c.setP(10);
    c.setI(20);
    c.setD(30);
    EXPECT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);

    c = Controller(0, 0, 0);
    c.setPID(10, 20, 30);
    EXPECT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);


    // it sets the timediff to 1/tickRate when time = 0 is given
    c = Controller(0, 0, 0, 0);
    EXPECT_EQ(c.timeDiff, 1.0/rtt::ai::Constants::TICK_RATE());

    // otherwise it sets the timediff
    c = Controller(0, 0, 0, 8);
    EXPECT_EQ(c.timeDiff, 8);

    c = Controller(0, 0, 0, 4, 1, 2, 3, 4);
    EXPECT_EQ(c.timeDiff, 4);
    EXPECT_EQ(c.initial_I, 1);
    EXPECT_EQ(c.initial_I2, 2);
    EXPECT_EQ(c.prev_error, 3);
    EXPECT_EQ(c.prev_error2, 4);

    c.setTimeDiff(33);
    EXPECT_EQ(c.timeDiff, 33);
    c.setInitial(22);
    EXPECT_EQ(c.initial_I, 22);
    c.setPrevErr(11);
    EXPECT_EQ(c.prev_error, 11);

    c.setP(1, 2);
    EXPECT_EQ(c.kP, 1);
    EXPECT_EQ(c.timeDiff, 2);

    c.setI(3, 4);
    EXPECT_EQ(c.kI, 3);
    EXPECT_EQ(c.timeDiff, 4);

    c.setD(5, 6);
    EXPECT_EQ(c.kD, 5);
    EXPECT_EQ(c.timeDiff, 6);

    c = Controller(0,0,0);
    auto val = c.controlPID({2, 2});
    EXPECT_EQ(val.x, 0);
    EXPECT_EQ(val.y, 0);


    c = Controller(0,0,0);
    val = c.controlPIR({2, 2}, {3,3});
    EXPECT_EQ(val.x, 0);
    EXPECT_EQ(val.y, 0);
}

TEST(ControllerTest, it_calculates_proper_pir) {
    Controller c;
    EXPECT_EQ(c.controlPIR(100, 100), 0);

    c = Controller(10, 20, 30);
    double expectedP = 120;
    EXPECT_EQ(c.controlP(12), expectedP);
    double expectedI = (1.0/rtt::ai::Constants::TICK_RATE()) * 12 * 20;
    EXPECT_EQ(c.controlI(12), expectedI);
    double expectedR = 30 * 12 * -1;
    EXPECT_EQ(c.controlR(12), expectedR);


    // clear controller otherwise timeDiff makes the values different
    c = Controller(10, 20, 30);
    EXPECT_EQ(c.controlPIR(12, 12), expectedP + expectedI + expectedR);
}


} // control
} // ai
} // rtt