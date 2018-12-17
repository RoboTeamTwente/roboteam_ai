//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RefereeData.h>
#include "../src/utilities/Referee.hpp"
#include "gtest/gtest.h"

TEST(RefereeTest, it_gets_and_sets_the_ref) {
    roboteam_msgs::RefereeData refereeData;
    refereeData.command.command = 33;
    rtt::ai::Referee::setRefereeData(refereeData);

    ASSERT_EQ(rtt::ai::Referee::getRefereeData().command.command, 33);

    refereeData.command.command = 123;
    rtt::ai::Referee::setRefereeData(refereeData);

    ASSERT_EQ(rtt::ai::Referee::getRefereeData().command.command, 123);
    ASSERT_EQ(rtt::ai::Referee::getPreviousRefereeData().command.command, 33);
}