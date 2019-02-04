//
// Created by mrlukasbos on 1-2-19.
//

#include <QtWidgets/QLayoutItem>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "TreeVisualizerWidget.h"
#include "QLayout"
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

TreeVisualizerWidget::TreeVisualizerWidget(MainWindow * parent)
        : QTreeWidget((QWidget *) parent)
{
    this->parent = parent;
    this->setColumnCount(2);
    this->setColumnWidth(0, 250);
}

// some widgets need to be updated regularly
void TreeVisualizerWidget::updateContents()
{
    // Iterate through all treeWidget items to update the status if needed
    QTreeWidgetItemIterator iter(this, QTreeWidgetItemIterator::All);
    while (*iter) {
        QTreeWidgetItem* widgetItem = *iter;
        if (treeItemMapping.find(widgetItem)!=treeItemMapping.end()) {
            bt::Node::Ptr item = treeItemMapping.at(widgetItem);
            QString status = QString::fromStdString(statusToString(item->getStatus()));
            if (widgetItem->text(1)!=status) {
                widgetItem->setText(1, status);
                widgetItem->setBackgroundColor(1, getColorForStatus(item->getStatus()));
            }
        }
        ++iter;
        delete widgetItem;
    }

    // initiate a redraw when the actual tree and the tree in the widget are not the same
    std::string currentTree = BTFactory::getFactory().getCurrentTree();
    if (QString::fromStdString(currentTree) != parent->getSelectStrategyText()) {
        hasCorrectTree = false;
        parent->setSelectStrategyText(QString::fromStdString(currentTree));
    }

    // if the tree did change, clear the treewidget and rebuild it
    if (!hasCorrectTree || BTFactory::getFactory().isInitialized()) {
        treeItemMapping.clear();
        this->clear();
        bt::BehaviorTree::Ptr tree = BTFactory::getFactory().getTree(BTFactory::getFactory().getCurrentTree());

        if (tree && tree->GetRoot()) {
            auto treeItemRoot = new QTreeWidgetItem(this);
            treeItemRoot->setText(0, QString::fromStdString(tree->GetRoot()->node_name()));
            treeItemRoot->setText(1, QString::fromStdString(statusToString(tree->GetRoot()->getStatus())));
            treeItemRoot->setBackgroundColor(1, getColorForStatus(tree->GetRoot()->getStatus()));

            addRootItem(tree->GetRoot(), treeItemRoot);
            this->expandAll();
            this->update();
        }
    }
}

/// Use recursion to iterate through the children of each node
void TreeVisualizerWidget::addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent)
{
    for (auto const& child : parent->getChildren()) {
        auto treeItemchild = new QTreeWidgetItem(QParent);
        treeItemchild->setText(0, QString::fromStdString(child->node_name()));
        treeItemchild->setText(1, QString::fromStdString(statusToString(child->getStatus())));

        std::pair<QTreeWidgetItem*, bt::Node::Ptr> pair{treeItemchild, child};
        treeItemMapping.insert(pair);

        treeItemchild->setBackgroundColor(1, getColorForStatus(child->getStatus()));
        QParent->addChild(treeItemchild);
        addRootItem(child, treeItemchild);
    }
}

/// returns a color for a given node status
QColor TreeVisualizerWidget::getColorForStatus(bt::Node::Status status) {
    switch (status) {
    case bt::Node::Status::Failure:return Qt::red;
    case bt::Node::Status::Running:return {"#99ff99"}; // light green
    case bt::Node::Status::Success:return {"#339933"}; // dark green
    case bt::Node::Status::Waiting:return Qt::gray;
    default:return Qt::white;
    }
}

void TreeVisualizerWidget::setHasCorrectTree(bool hasCorrectTree) {
    TreeVisualizerWidget::hasCorrectTree = hasCorrectTree;
}

}
}
}