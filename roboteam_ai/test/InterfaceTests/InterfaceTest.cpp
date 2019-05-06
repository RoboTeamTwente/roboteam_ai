//
// Created by mrlukasbos on 18-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/api/Output.h>

TEST(InterfaceTest, the_interface_values_work) {
    using iv = rtt::ai::interface::Output;

    auto pidnum = rtt::ai::pidVals(12, -18, 33333.2);
    iv::setNumTreePid(pidnum);
    EXPECT_EQ(iv::getNumTreePid(), pidnum);

    auto pidbasic = rtt::ai::pidVals(12, -22, 12.2);
    iv::setBasicPid(pidbasic);
    EXPECT_EQ(iv::getBasicPid(), pidbasic);

    auto pidforce= rtt::ai::pidVals(8888, -7, 8.2);
    iv::setForcePid(pidforce);
    EXPECT_EQ(iv::getForcePid(), pidforce);
}

