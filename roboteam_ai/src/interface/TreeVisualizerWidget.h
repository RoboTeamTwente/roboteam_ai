//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <QtWidgets/QTreeWidget>
#include <roboteam_ai/src/bt/Node.hpp>
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

class TreeVisualizerWidget : public QTreeWidget {
    Q_OBJECT
    FRIEND_TEST(TreeVisualizerTest, it_toggles_our_color_param);
private:
    QColor getColorForStatus(bt::Node::Status status);
    void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent);
    std::map<QTreeWidgetItem *, bt::Node::Ptr> treeItemMapping;
    bool hasCorrectTree = false;
    MainWindow * parent = nullptr;
public:
    bool isHasCorrectTree() const;
    void setHasCorrectTree(bool hasCorrectTree);
public:
    explicit TreeVisualizerWidget(MainWindow * parent);
public slots:
    void updateContents();
};

}
}
}
#endif //ROBOTEAM_AI_TREEVISUALIZER_H
