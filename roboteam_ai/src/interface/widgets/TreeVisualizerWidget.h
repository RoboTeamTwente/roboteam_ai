//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <QtWidgets/QTreeWidget>
#include <roboteam_ai/src/bt/Node.hpp>
#include "roboteam_ai/src/bt/BehaviorTree.hpp"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow;
class TreeVisualizerWidget : public QTreeWidget {
    Q_OBJECT
    FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);
    FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);
private:
    QColor getColorForStatus(bt::Node::Status status);
    void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent);
    std::map<QTreeWidgetItem *, bt::Node::Ptr> treeItemMapping;
    bool hasCorrectTree = false;
    MainWindow * parent = nullptr;
    unsigned long long mostTicks = 0;
public:
    void setHasCorrectTree(bool hasCorrectTree);
    explicit TreeVisualizerWidget(MainWindow * parent);
public slots:
    void updateContents(bt::BehaviorTree::Ptr tree);
    void populateRow(bt::Node::Ptr node, QTreeWidgetItem* row, bool isUpdate = false);
};

}
}
}
#endif //ROBOTEAM_AI_TREEVISUALIZER_H
