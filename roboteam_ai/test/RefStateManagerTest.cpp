//
// Created by rolf on 25-10-18.
//
#include "gtest/gtest.h"
#include "../src/utilities/RefStateManager.hpp"
namespace rtt{
    namespace ai{
        namespace RSMTest{
        class TestLeaf :public bt::Leaf{
        private:
            bt::Node::Status statusTest;
        public:
            TestLeaf(){
                this->statusTest=bt::Node::Status::Running;
            }
            Status Update() override{
                return statusTest;
            }
            void setStatusTest(Status statusTest){
                this->statusTest=statusTest;
            }
        };
        }
    }
}
namespace r=rtt::ai;
// You should see the following warnings when running this test. If not, something is probably going wrong!
//[ WARN][ros.roboteam_ai.RefStateManager]: Have not yet received a ref command, so not executing any strategy tree.
//[ INFO][ros.roboteam_ai.RefStateManager]: Botcount changed from 4294967295 to 0
//[ INFO][ros.roboteam_ai.RefStateManager]: RefState switch detected : PREPARE_KICKOFF_US -> NORMAL_START | DO_KICKOFF : rtt_bob/KickoffWithChipStrategy
//[ WARN][ros.roboteam_ai.RefStateManager]: Keeper ID changed from 0 to 1
//[ INFO][ros.roboteam_ai.RefStateManager]: Botcount changed from 0 to 1
TEST(RefStateManager,refstatemanagertest){
    roboteam_msgs::World world;
    rtt::ai::World::set_world(world);
    rtt::ai::Referee::Reset();
    // Need clean memory because otherwise this test breaks because of the referee and world tests

    std::shared_ptr<r::RefStateManager> rsm = std::make_shared<r::RefStateManager>();
    ASSERT_EQ(rsm->node_name(), "RefStateManager");
    EXPECT_EQ(rsm->Update(), bt::Node::Status::Running);


    rtt::ai::RSMTest::TestLeaf root;
    std::shared_ptr<rtt::ai::RSMTest::TestLeaf> rootPtr = std::make_shared<rtt::ai::RSMTest::TestLeaf>(root);
    bt::BehaviorTree treeA(rootPtr);
    std::shared_ptr<bt::BehaviorTree> treeAPtr = std::make_shared<bt::BehaviorTree>(treeA);

    rtt::ai::RSMTest::TestLeaf rootTwo;
    std::shared_ptr<rtt::ai::RSMTest::TestLeaf> rootTwoPtr = std::make_shared<rtt::ai::RSMTest::TestLeaf>(rootTwo);
    bt::BehaviorTree treeB(rootTwoPtr);
    std::shared_ptr<bt::BehaviorTree> treeBPtr = std::make_shared<bt::BehaviorTree>(treeB);


    rtt::ai::RSMTest::TestLeaf rootThree;
    std::shared_ptr<rtt::ai::RSMTest::TestLeaf> rootThreePtr = std::make_shared<rtt::ai::RSMTest::TestLeaf>(rootThree);
    bt::BehaviorTree treeC(rootThreePtr);
    std::shared_ptr<bt::BehaviorTree> treeCPtr = std::make_shared<bt::BehaviorTree>(treeC);

    rsm->AddStrategy(r::RefGameState::PREPARE_KICKOFF_US, treeAPtr);
    rsm->AddStrategy(r::RefGameState::NORMAL_START, treeBPtr);
    rsm->AddStrategy(r::RefGameState::DO_KICKOFF, treeCPtr);

    EXPECT_EQ(rsm->getCurrentStrategyTreeName(),
              "Current strategy not found? Check if a RefState was sent and if all trees were set correctly."); // This function is for debug purposes but it is nice to test if it works

    roboteam_msgs::RefereeData msg, msgTwo;
    msg.command.command = 4;//PREPARE_KICKOFF_US
    r::Referee::set(msg);
    EXPECT_EQ(rsm->Update(), bt::Node::Status::Running);

    msgTwo.command.command = 2;//NORMAL_START
    msgTwo.us.goalie = 1;//Setting new goalie
    r::Referee::set(msgTwo);
    EXPECT_EQ(rsm->Update(), bt::Node::Status::Running);
    //treeCPtr->setStatus(bt::Node::Status::Success);
    rootThreePtr->setStatusTest(bt::Node::Status::Success);
    EXPECT_EQ(rsm->Update(), bt::Node::Status::Running);
    EXPECT_EQ(rsm->Update(), bt::Node::Status::Running);

    //Change the world count
    roboteam_msgs::WorldRobot bot;
    world.us.push_back(bot);
    rtt::ai::World::set_world(world);
    EXPECT_EQ(rsm->Update(),bt::Node::Status::Running);
    }
