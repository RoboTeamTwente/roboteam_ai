//
// Created by rolf on 25-10-18.
//
#include "gtest/gtest.h"
#include "../src/utilities/RefStateManager.hpp"
namespace rtt{
    namespace ai{
        namespace RSMTest{
        class TestLeaf :public bt::Leaf{
        public:
            TestLeaf(){
                this->status=bt::Node::Status::Running;
            }
            Status Update() override{
                return status;
            }
            Status status;
            void setStatus(Status status){
                this->status=status;
            }
        };
        }
    }
}
namespace r=rtt::ai;
// You should see the following warnings when running this test. If not, something is going wrong!
//[ WARN][ros.roboteam_ai.RefStateManager]: Have not yet received a ref command, so not executing any strategy tree.
//[ INFO][ros.roboteam_ai.RefStateManager]: Botcount changed from 4294967295 to 0

TEST(RefStateManager,refstatemanagertest){
    std::shared_ptr<r::RefStateManager> rsm= std::make_shared<r::RefStateManager>();
    ASSERT_EQ(rsm->node_name(),"RefStateManager");
    EXPECT_EQ(rsm->Update(),bt::Node::Status::Running);

    bt::BehaviorTree treeA,treeB,treeC;

    rtt::ai::RSMTest::TestLeaf root;
    treeA.SetRoot(std::make_shared<rtt::ai::RSMTest::TestLeaf>(root));
    std::shared_ptr<bt::BehaviorTree> treeAPtr=std::make_shared<bt::BehaviorTree>(treeA);
    treeAPtr->node_name()="treeA";

    rtt::ai::RSMTest::TestLeaf rootTwo;
    treeB.SetRoot(std::make_shared<rtt::ai::RSMTest::TestLeaf>(rootTwo));
    std::shared_ptr<bt::BehaviorTree> treeBPtr=std::make_shared<bt::BehaviorTree>(treeB);
    treeBPtr->node_name()="treeB";
    rsm->AddStrategy(r::RefGameState::PREPARE_KICKOFF_THEM,treeAPtr);
    rsm->AddStrategy(r::RefGameState::NORMAL_START,treeBPtr);
    EXPECT_EQ(rsm->getCurrentStrategyTreeName(),"Current strategy not found? Check if a RefState was sent and if all trees were set correctly."); // This function is for debug purposes but it is nice to test if it works

    roboteam_msgs::RefereeData msg,msgTwo;
    msg.command.command=4;//PREPARE_KICKOFF_THEM
    r::Referee::set(msg);
    EXPECT_EQ(rsm->Update(),bt::Node::Status ::Running);

    msgTwo.command.command=2;//NORMAL_START
    r::Referee::set(msgTwo);
    EXPECT_EQ(rsm->Update(),bt::Node::Status::Running);

}
