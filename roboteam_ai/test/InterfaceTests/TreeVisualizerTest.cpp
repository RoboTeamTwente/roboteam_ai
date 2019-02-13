//
// Created by mrlukasbos on 4-2-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/TreeVisualizerWidget.h>
#include <roboteam_ai/src/interface/mainWindow.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>

namespace rtt {
namespace ai {
namespace interface {

TEST(TreeVisualizerTest, it_properly_displays_trees) {
    BTFactory factory;
    factory.init();
    BTFactory::setCurrentTree("randomStrategy");
    auto window = std::make_shared<MainWindow>();
    TreeVisualizerWidget * treeVis = window->treeWidget;

    // it initializes to false
    EXPECT_FALSE(treeVis->hasCorrectTree);
    EXPECT_TRUE(treeVis->treeItemMapping.empty());

    treeVis->updateContents();
    EXPECT_TRUE(treeVis->hasCorrectTree);
    EXPECT_EQ(treeVis->treeItemMapping.size(), 24);

    std::map<QTreeWidgetItem *, bt::Node::Ptr>::iterator it;
    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string node, tree, status;

        tree = it->first->text(0).toStdString();
        status = it->first->text(1).toStdString();
        node = it->second->node_name();

        // check if the pairs of layout and nodes are properly connected
        EXPECT_EQ(tree, node);
        EXPECT_EQ(status, bt::statusToString(it->second->getStatus()));

        it->second->terminate(bt::Node::Status::Running);
    }

    treeVis->updateContents();
    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string node, tree, status;

        tree = it->first->text(0).toStdString();
        status = it->first->text(1).toStdString();
        node = it->second->node_name();

        // check if the pairs of layout and nodes are properly connected
        EXPECT_EQ(tree, node);
        EXPECT_EQ(status, bt::statusToString(it->second->getStatus()));
        EXPECT_TRUE(status == "Failure" || status == "Waiting");
    }

    // check if it properly switches a strategy
    BTFactory::setCurrentTree("haltStrategy");
    treeVis->updateContents();
    EXPECT_TRUE(treeVis->hasCorrectTree);

    EXPECT_EQ(treeVis->treeItemMapping.size(), 27);
    for (it = treeVis->treeItemMapping.begin(); it != treeVis->treeItemMapping.end(); it++) {
        std::string nodeTrace, treeTrace, statusTrace;

        treeTrace = it->first->text(0).toStdString();
        statusTrace = it->first->text(1).toStdString();
        nodeTrace = it->second->node_name();

        // check if the pairs of layout and nodes are properly connected
        EXPECT_EQ(treeTrace, nodeTrace);
        EXPECT_EQ(statusTrace, bt::statusToString(it->second->getStatus()));
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