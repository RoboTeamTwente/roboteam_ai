//
// Created by mrlukasbos on 18-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

TEST(InterfaceTest, the_interface_values_work) {
    using iv = rtt::ai::interface::InterfaceValues;
    {
        iv::setLuthP(12);
        iv::setLuthI(-18);
        iv::setLuthD(33333.2);
    }
    {
        EXPECT_EQ(iv::getLuthP(), 12);
        EXPECT_EQ(iv::getLuthI(), -18);
        EXPECT_EQ(iv::getLuthD(), 33333.2);
    }
}

