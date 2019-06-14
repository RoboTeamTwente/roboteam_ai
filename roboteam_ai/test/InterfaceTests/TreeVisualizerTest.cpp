//
// Created by mrlukasbos on 4-2-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/widgets/TreeVisualizerWidget.h>
#include <roboteam_ai/src/interface/widgets/mainWindow.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include <roboteam_ai/src/skills/Halt.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>

namespace rtt {
namespace ai {
namespace interface {

TEST(TreeVisualizerTest, it_properly_displays_trees) {

    auto field = testhelpers::FieldHelper::generateField();
    // put in a world to make robotdealer halt work etc.
    auto worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 0, true, field);
    rtt::ai::world::world->updateWorld(worldmsg.first);
std::cout << "a";
    BTFactory factory;
    factory.makeTrees();
    BTFactory::setCurrentTree("halt_strategy");
    auto window = std::make_shared<MainWindow>();
    TreeVisualizerWidget * treeVis = window->treeWidget;
        std::cout << "b";

        // it initializes to false
    EXPECT_FALSE(treeVis->hasCorrectTree);
    EXPECT_TRUE(treeVis->treeItemMapping.empty());

    treeVis->updateContents(BTFactory::getTree(BTFactory::getCurrentTree()));
    EXPECT_TRUE(treeVis->hasCorrectTree);
    EXPECT_EQ(treeVis->treeItemMapping.size(), 18);
        std::cout << "c";

        std::map<QTreeWidgetItem *, bt::Node::Ptr>::iterator it;
    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string node, tree, status;

        tree = it->first->text(0).toStdString();
        status = it->first->text(1).toStdString();
        node = it->second->node_name();

        // check if the pairs of layout and nodes are properly connected
        auto properties = std::make_shared<bt::Blackboard>();
        bt::Node::Ptr n = std::make_shared<rtt::ai::Halt>("halt", properties);
        EXPECT_EQ(tree, node);
        EXPECT_EQ(status, n->status_print(it->second->getStatus()));

        it->second->terminate(bt::Node::Status::Running);
    }
        std::cout << "d";

    treeVis->updateContents(BTFactory::getTree(BTFactory::getCurrentTree()));
    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string node, tree, status;

        tree = it->first->text(0).toStdString();
        status = it->first->text(1).toStdString();
        node = it->second->node_name();

        auto properties = std::make_shared<bt::Blackboard>();
        bt::Node::Ptr n = std::make_shared<rtt::ai::Halt>("halt", properties);
        // check if the pairs of layout and nodes are properly connected
        EXPECT_EQ(tree, node);
        EXPECT_EQ(status, n->status_print(it->second->getStatus()));
        EXPECT_TRUE(status == "Failure" || status == "Waiting");
    }
        std::cout << "e";

    // check if it properly switches a strategy
    BTFactory::setCurrentTree("interface_drive_strategy");
    treeVis->updateContents(BTFactory::getTree(BTFactory::getCurrentTree()));
    EXPECT_TRUE(treeVis->hasCorrectTree);
    EXPECT_EQ(treeVis->treeItemMapping.size(), 18);
        std::cout << "f";

    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string nodeTrace, treeTrace, statusTrace;

        treeTrace = it->first->text(0).toStdString();
        statusTrace = it->first->text(1).toStdString();
        nodeTrace = it->second->node_name();

        // check if the pairs of layout and nodes are properly connected
        auto properties = std::make_shared<bt::Blackboard>();
        bt::Node::Ptr n = std::make_shared<rtt::ai::Halt>("halt", properties);
        EXPECT_EQ(treeTrace, nodeTrace);
        EXPECT_EQ(statusTrace, n->status_print(it->second->getStatus()));
    }
}

TEST(TreeVisualizerTest, it_sets_proper_color_for_status) {
    auto window = std::make_shared<MainWindow>();
    TreeVisualizerWidget * treeVis = window->treeWidget;
    EXPECT_EQ(treeVis->getColorForStatus(bt::Node::Status::Failure), Qt::red);
    EXPECT_EQ(treeVis->getColorForStatus(bt::Node::Status::Success), QColor("#66ff66"));
    EXPECT_EQ(treeVis->getColorForStatus(bt::Node::Status::Running), QColor("#006600"));
    EXPECT_EQ(treeVis->getColorForStatus(bt::Node::Status::Waiting), Qt::darkGray);
}



} // interface
} // ai
} // rtt