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

    c = Controller(100, 200, 300);
    ASSERT_EQ(c.controlP(12), 1200);
    ASSERT_EQ(c.controlI(12), 0);
    ASSERT_EQ(c.controlD(12), 0);
    ASSERT_EQ(c.controlPID(100), 1200);

}
}
}
}