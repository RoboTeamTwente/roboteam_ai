//
// Created by rolf on 25-10-18.
//
#include "gtest/gtest.h"
#include "../src/utilities/Referee.hpp"

TEST(Referee,TwoStateSwitch){
    //Test isTwoState (and the mapping)
    EXPECT_TRUE(rtt::ai::Referee::isTwoState(rtt::ai::RefGameState::PREPARE_KICKOFF_US, rtt::ai::RefGameState::NORMAL_START));
    EXPECT_TRUE(rtt::ai::Referee::isTwoState(rtt::ai::RefGameState::PREPARE_KICKOFF_THEM, rtt::ai::RefGameState::NORMAL_START));
    EXPECT_TRUE(rtt::ai::Referee::isTwoState(rtt::ai::RefGameState::PREPARE_PENALTY_US, rtt::ai::RefGameState::NORMAL_START));
    EXPECT_TRUE(rtt::ai::Referee::isTwoState(rtt::ai::RefGameState::PREPARE_PENALTY_THEM, rtt::ai::RefGameState::NORMAL_START));


    for (rtt::ai::RefGameState state : rtt::ai::ALL_REFSTATES){
        EXPECT_TRUE(rtt::ai::Referee::isTwoState(state,rtt::ai::RefGameState::DIRECT_FREE_THEM));
        EXPECT_TRUE(rtt::ai::Referee::isTwoState(state,rtt::ai::RefGameState::DIRECT_FREE_US));
        EXPECT_TRUE(rtt::ai::Referee::isTwoState(state,rtt::ai::RefGameState::INDIRECT_FREE_THEM));
        EXPECT_TRUE(rtt::ai::Referee::isTwoState(state,rtt::ai::RefGameState::INDIRECT_FREE_US));
    }

    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::optional<rtt::ai::RefGameState >(rtt::ai::RefGameState::PREPARE_KICKOFF_US), rtt::ai::RefGameState::NORMAL_START),rtt::ai::RefGameState::DO_KICKOFF);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::optional<rtt::ai::RefGameState >(rtt::ai::RefGameState::PREPARE_KICKOFF_THEM), rtt::ai::RefGameState::NORMAL_START),rtt::ai::RefGameState::DEFEND_KICKOFF);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::optional<rtt::ai::RefGameState >(rtt::ai::RefGameState::PREPARE_PENALTY_THEM), rtt::ai::RefGameState::NORMAL_START),rtt::ai::RefGameState::DEFEND_PENALTY);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::optional<rtt::ai::RefGameState >(rtt::ai::RefGameState::PREPARE_PENALTY_US), rtt::ai::RefGameState::NORMAL_START),rtt::ai::RefGameState::DO_PENALTY);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::none, rtt::ai::RefGameState::INDIRECT_FREE_THEM),rtt::ai::RefGameState::INDIRECT_FREE_THEM);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::none, rtt::ai::RefGameState::INDIRECT_FREE_US),rtt::ai::RefGameState::INDIRECT_FREE_US);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::none, rtt::ai::RefGameState::DIRECT_FREE_THEM),rtt::ai::RefGameState::DIRECT_FREE_THEM);
    EXPECT_EQ(rtt::ai::Referee::getFirstState(boost::none, rtt::ai::RefGameState::DIRECT_FREE_US),rtt::ai::RefGameState ::DIRECT_FREE_US);

}

TEST(Referee,GettingAndSetting){
    namespace r=rtt::ai;

    EXPECT_FALSE(r::Referee::hasReceivedFirstCommand());
    roboteam_msgs::RefereeData refCmdOne,refCmdTwo,refCmdThree;
    refCmdOne.stage_time_left=10;
    refCmdOne.stage.stage=1; //NORMAL_FIRST_HALF
    refCmdOne.command.command=4;//PREPARE_KICKOFF_US
    r::Referee::set(refCmdOne);

    EXPECT_EQ(r::Referee::get().stage.stage,refCmdOne.stage.stage);
    EXPECT_EQ(r::Referee::getState(),r::RefGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(r::Referee::getCurrentRefCommand(),r::RefGameState::PREPARE_KICKOFF_US);
    EXPECT_FALSE(r::Referee::getPreviousRefCommand());
    EXPECT_TRUE(r::Referee::hasReceivedFirstCommand());
    EXPECT_EQ(r::Referee::getTimeLeft("NORMAL_FIRST_HALF"),10);
    EXPECT_NE(r::Referee::getTimeLeft("NORMAL_SECOND_HALF"),10);

    refCmdTwo.stage_time_left=8;
    refCmdTwo.stage.stage=1; //NORMAL_FIRST_HALF
    refCmdTwo.command.command=2; //NORMAL_START
    refCmdTwo.us.score=2;
    refCmdTwo.them.score=3;
    r::Referee::set(refCmdTwo);

    EXPECT_EQ(r::Referee::get().stage.stage,refCmdTwo.stage.stage);
    EXPECT_EQ(r::Referee::getState(),r::RefGameState::NORMAL_START);
    EXPECT_EQ(r::Referee::getCurrentRefCommand(),r::RefGameState::NORMAL_START);
    EXPECT_EQ(r::Referee::getPreviousRefCommand(),r::RefGameState::PREPARE_KICKOFF_US);
    EXPECT_TRUE(r::Referee::hasReceivedFirstCommand());
    EXPECT_EQ(r::Referee::getTimeLeft("NORMAL_FIRST_HALF"),8);
    EXPECT_NE(r::Referee::getTimeLeft("NORMAL_SECOND_HALF"),10);
    std::pair<int,int> score (2,3);
    EXPECT_EQ(r::Referee::currentScore(),score);
    EXPECT_EQ(r::Referee::getFirstState(),r::RefGameState::DO_KICKOFF);
    EXPECT_EQ(r::Referee::getExtendedState(),r::RefGameState::DO_KICKOFF);

    refCmdThree.stage_time_left=6;
    refCmdThree.stage.stage=4; //NORMAL_SECOND_HALF
    refCmdThree.command.command=0; //HALT
    refCmdThree.us.score=3;
    refCmdThree.them.score=3;
    r::Referee::set(refCmdThree);

    EXPECT_EQ(r::Referee::get().stage.stage,refCmdThree.stage.stage);
    EXPECT_EQ(r::Referee::getState(),r::RefGameState::HALT);
    EXPECT_EQ(r::Referee::getCurrentRefCommand(),r::RefGameState::HALT);
    EXPECT_EQ(r::Referee::getPreviousRefCommand(),r::RefGameState::NORMAL_START);
    EXPECT_TRUE(r::Referee::hasReceivedFirstCommand());
    EXPECT_EQ(r::Referee::getTimeLeft("NORMAL_FIRST_HALF"),8);
    EXPECT_EQ(r::Referee::getTimeLeft("NORMAL_SECOND_HALF"),6);
    score ={3,3};
    EXPECT_EQ(r::Referee::currentScore(),score);
    EXPECT_EQ(r::Referee::getFirstState(),boost::none);
    EXPECT_EQ(r::Referee::getExtendedState(),r::RefGameState::HALT);
}