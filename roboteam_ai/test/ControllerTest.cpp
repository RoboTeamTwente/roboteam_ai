//
// Created by mrlukasbos on 11-1-19.
//
#include <gtest/gtest.h>
#include "../src/control/Controller.h"

namespace rtt {
namespace ai {
namespace control {

TEST(ControllerTest, it_calculates_proper_pid) {

    // Empty controller
    Controller c;
    ASSERT_EQ(c.controlPID(100), 0);

    // controller with P, I and D
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

    c = Controller(0, 0, 0);
    c.setPID(10, 20, 30);
    ASSERT_EQ(c.controlPID(12), expectedP + expectedI + expectedD);


    // it sets the timediff to 1/tickRate when time = 0 is given
    c = Controller(0, 0, 0, 0);
    ASSERT_EQ(c.timeDiff, 1.0/rtt::ai::constants::tickRate);

    // otherwise it sets the timediff
    c = Controller(0, 0, 0, 8);
    ASSERT_EQ(c.timeDiff, 8);

    c = Controller(0, 0, 0, 4, 1, 2, 3, 4);
    ASSERT_EQ(c.timeDiff, 4);
    ASSERT_EQ(c.initial_I, 1);
    ASSERT_EQ(c.initial_I2, 2);
    ASSERT_EQ(c.prev_error, 3);
    ASSERT_EQ(c.prev_error2, 4);

    c.setTimeDiff(33);
    ASSERT_EQ(c.timeDiff, 33);
    c.setInitial(22);
    ASSERT_EQ(c.initial_I, 22);
    c.setPrevErr(11);
    ASSERT_EQ(c.prev_error, 11);

    c.setP(1, 2);
    ASSERT_EQ(c.kP, 1);
    ASSERT_EQ(c.timeDiff, 2);

    c.setI(3, 4);
    ASSERT_EQ(c.kI, 3);
    ASSERT_EQ(c.timeDiff, 4);

    c.setD(5, 6);
    ASSERT_EQ(c.kD, 5);
    ASSERT_EQ(c.timeDiff, 6);

    c = Controller(0,0,0);
    auto val = c.controlPID({2, 2});
    ASSERT_EQ(val.x, 0);
    ASSERT_EQ(val.y, 0);


    c = Controller(0,0,0);
    val = c.controlPIR({2, 2}, {3,3});
    ASSERT_EQ(val.x, 0);
    ASSERT_EQ(val.y, 0);
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