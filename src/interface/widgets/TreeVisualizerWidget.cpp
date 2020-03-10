/*
 *
 * This widget visualizes a behaviour (strategy) tree.
 *
 * Update contents updates the statuses, ticktimes and amount of ticks for each node.
 * The nodes in the tree are not refreshed until the whole tree is invalidated.
 *
 */

#include "interface/widgets/TreeVisualizerWidget.h"
#include <treeinterp/BTFactory.h>
#include <QtWidgets/QLayoutItem>
#include <utilities/GameStateManager.hpp>
#include "interface/widgets/mainWindow.h"

namespace rtt::ai::interface {

TreeVisualizerWidget::TreeVisualizerWidget(MainWindow *parent) : QTreeWidget((QWidget *)parent) {
    this->parent = parent;
    this->setColumnCount(4);
    this->setColumnWidth(0, 250);

    QTreeWidgetItem *header = this->headerItem();
    header->setText(0, "Node");
    header->setText(1, "Status");
    header->setText(2, "Last tick");
    header->setText(3, "# of ticks");
}

// some widgets need to be updated regularly
void TreeVisualizerWidget::updateContents(bt::BehaviorTree::Ptr tree) {
    // Iterate through all treeWidget items to update the status if needed
    QTreeWidgetItemIterator iter(this, QTreeWidgetItemIterator::All);
    while (*iter) {
        QTreeWidgetItem *widgetItem = *iter;
        if (treeItemMapping.find(widgetItem) != treeItemMapping.end()) {
            populateRow(treeItemMapping.at(widgetItem), widgetItem, true);
        }
        ++iter;
    }

    GameState state = GameStateManager::getCurrentGameState();
    if (recentGameState.strategyName != state.strategyName) {
        hasCorrectTree = false;
        recentGameState = state;
    }

    // if the tree did change, clear the treewidget and rebuild it
    if (!hasCorrectTree) {
        treeItemMapping.clear();
        this->clear();
        mostTicks = 0;

        if (tree && tree->GetRoot()) {
            auto treeItemRoot = new QTreeWidgetItem(this);
            populateRow(tree->GetRoot(), treeItemRoot);

            // recursively draw the nodes
            addRootItem(tree->GetRoot(), treeItemRoot);

            this->expandAll();
            this->update();
            hasCorrectTree = true;
        }
    }
}

/// Use recursion to iterate through the children of each node
void TreeVisualizerWidget::addRootItem(bt::Node::Ptr parent, QTreeWidgetItem *QParent) {
    for (auto const &child : parent->getChildren()) {
        auto treeItemchild = new QTreeWidgetItem(QParent);
        populateRow(child, treeItemchild);
        QParent->addChild(treeItemchild);
        addRootItem(child, treeItemchild);
    }
}

// update the contents in a row of the treewidget
void TreeVisualizerWidget::populateRow(bt::Node::Ptr node, QTreeWidgetItem *row, bool isUpdate) {
    // if the row is updated we don't need to change node names
    // also insert the pair into treeItemMapping
    if (!isUpdate) {
        row->setText(0, QString::fromStdString(node->node_name()));
        std::pair<QTreeWidgetItem *, bt::Node::Ptr> pair{row, node};
        treeItemMapping.insert(pair);
    }

    // Check if the status is changed and update it if needed.
    QString status = QString::fromStdString(node->status_print(node->getStatus()));  // statusToString(node->getStatus()));
    if (row->text(1) != status) {
        row->setText(1, status);
        row->setForeground(1, getColorForStatus(node->getStatus()));
    }

    row->setText(3, QString::number(node->getAmountOfTicks(), 'f', 0));
}

/// returns a color for a given node status
QColor TreeVisualizerWidget::getColorForStatus(bt::Node::Status status) {
    switch (status) {
        case bt::Node::Status::Failure:
            return Qt::red;
        case bt::Node::Status::Running:
            return {"#006600"};  // dark green
        case bt::Node::Status::Success:
            return {"#66ff66"};  // bright green
        case bt::Node::Status::Waiting:
            return Qt::darkGray;
        default:
            return Qt::white;
    }
}

// make it possible to invalidate the tree to force a reload
void TreeVisualizerWidget::setHasCorrectTree(bool hasCorrectTree) { TreeVisualizerWidget::hasCorrectTree = hasCorrectTree; }

void TreeVisualizerWidget::invalidateTree() { TreeVisualizerWidget::hasCorrectTree = false; }

}  // namespace rtt::ai::interface