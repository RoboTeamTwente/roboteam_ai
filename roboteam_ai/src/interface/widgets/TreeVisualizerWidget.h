//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <roboteam_ai/src/bt/Node.hpp>
#include <QtWidgets/QTreeWidget>
#include "../../bt/BehaviorTree.hpp"

namespace rtt {
namespace ai {

struct GameState;

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
    GameState * recentGameState;
public:
    explicit TreeVisualizerWidget(MainWindow * parent);
    void setHasCorrectTree(bool hasCorrectTree);
public slots:
    void invalidateTree();
    void updateContents(bt::BehaviorTree::Ptr tree);
    void populateRow(bt::Node::Ptr node, QTreeWidgetItem* row, bool isUpdate = false);
};

}
}
}
#endif //ROBOTEAM_AI_TREEVISUALIZER_H
