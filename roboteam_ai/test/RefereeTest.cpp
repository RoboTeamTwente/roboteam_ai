//
// Created by rolf on 25-10-18.
//
#include "gtest/gtest.h"
#include "../src/utilities/Referee.hpp"

//Assumes that ALL_REFSTATES is correct. If not, a lot more tests and things should break.
//Tests whether the mapping to and from string and to and from integer is correct and consistent
TEST(MapsAreCorrect,Referee){
    for (rtt::ai::RefGameState state : rtt::ai::ALL_REFSTATES){
        std::string strState= rtt::ai::refStateToString(state);
        boost::optional<rtt::ai::RefGameState> derivedState=rtt::ai::stringToRefState(strState);
        EXPECT_TRUE(derivedState);
        EXPECT_EQ(state,*derivedState);
        std::string doubleDerivedState=rtt::ai::refStateToString(*derivedState);
        EXPECT_EQ(strState,doubleDerivedState);

        //The integers are only used for communication with the actual Ref, so we exclude our custom Refstates from testing.
        if (state!=rtt::ai::RefGameState::DO_KICKOFF
          &&state!=rtt::ai::RefGameState::DO_PENALTY
          &&state!=rtt::ai::RefGameState::DEFEND_KICKOFF
          &&state!=rtt::ai::RefGameState::DEFEND_PENALTY) {
            boost::optional<int> intState = rtt::ai::fromRefState(state);
            EXPECT_TRUE(intState);
            boost::optional<rtt::ai::RefGameState> derivedIntState = rtt::ai::toRefState(*intState);
            EXPECT_TRUE(derivedIntState);
            EXPECT_EQ(state, *derivedIntState);
            boost::optional<int> doubleDerivedIntState = rtt::ai::fromRefState(*derivedIntState);
            EXPECT_TRUE(doubleDerivedIntState);
            EXPECT_EQ(*intState, *doubleDerivedIntState);
        }
    }
}