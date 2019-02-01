//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <QtWidgets/QTreeWidget>
#include <roboteam_ai/src/bt/Node.hpp>

class TreeVisualizerWidget : public QTreeWidget {
    Q_OBJECT
private:
    QColor getColorForStatus(bt::Node::Status status);
    void clearLayout(QLayout *layout);
    void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent);
    std::map<QTreeWidgetItem *, bt::Node::Ptr> treeItemMapping;


public:
    explicit TreeVisualizerWidget(QWidget * parent);
    void updateContents();
};

#endif //ROBOTEAM_AI_TREEVISUALIZER_H
