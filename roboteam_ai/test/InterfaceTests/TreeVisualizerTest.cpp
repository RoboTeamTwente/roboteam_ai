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

    BTFactory::setCurrentTree("haltStrategy");

    auto window = std::make_shared<MainWindow>();
    TreeVisualizerWidget * treeVis = window->treeWidget;

    ASSERT_FALSE(treeVis->hasCorrectTree);
}


TEST(TreeVisualizerTest, it_sets_proper_color_for_status) {
    auto window = std::make_shared<MainWindow>();
    TreeVisualizerWidget * treeVis = window->treeWidget;
    ASSERT_EQ(treeVis->getColorForStatus(bt::Node::Status::Failure), Qt::red);
    ASSERT_EQ(treeVis->getColorForStatus(bt::Node::Status::Success), QColor("#339933"));
    ASSERT_EQ(treeVis->getColorForStatus(bt::Node::Status::Running), QColor("#99ff99"));
    ASSERT_EQ(treeVis->getColorForStatus(bt::Node::Status::Waiting), Qt::gray);
}

}
}
}