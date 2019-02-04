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

}

}
}
}