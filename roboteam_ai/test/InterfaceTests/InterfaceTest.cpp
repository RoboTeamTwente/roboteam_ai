//
// Created by mrlukasbos on 18-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

TEST(InterfaceTest, the_interface_values_work) {
    using iv = rtt::ai::interface::InterfaceValues;
    {
        iv::setLuthPosP(12);
        iv::setNumTreePosI(- 18);
        iv::setNumTreePosD(33333.2);
    }
    {
        EXPECT_EQ(iv::setNumTreePosP(), 12);
        EXPECT_EQ(iv::getNumTreePosI(), -18);
        EXPECT_EQ(iv::getNumTreePosD(), 33333.2);
    }
}

