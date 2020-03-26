/*
 *
 * This widget visualizes a behaviour (strategy) tree.
 *
 * Update contents updates the statuses, ticktimes and amount of ticks for each node.
 * The nodes in the tree are not refreshed until the whole tree is invalidated.
 *
 */

#include "stp/Play.hpp"
#include "stp/Role.hpp"
#include "interface/widgets/STPVisualizerWidget.h"
#include <treeinterp/BTFactory.h>
#include <QtWidgets/QLayoutItem>
#include <include/roboteam_ai/bt/Node.h>
#include "interface/widgets/mainWindow.h"

namespace rtt::ai::interface {

    STPVisualizerWidget::STPVisualizerWidget(MainWindow *parent) : QTextEdit(parent) {
        this->parent = parent;
        this->setReadOnly(true);
    }

    QColor STPVisualizerWidget::getColorForStatus(stp::Status status) {
        switch (status) {
            case stp::Status::Failure:
                return Qt::red;
            case stp::Status::Running:
                return {"#006600"};  // dark green
            case stp::Status::Success:
                return {"#66ff66"};  // bright green
            case stp::Status::Waiting:
                return Qt::darkGray;
            default:
                return Qt::white;
        }
    }

    void STPVisualizerWidget::updateContents(stp::Play *currentPlay) {
        updateContent.str("");
        displayPlay(currentPlay);
    }

    void STPVisualizerWidget::displayPlay(stp::Play * currentPlay) {
        updateContent << currentPlay->getName() << "<br>";
        for (auto& [role, state] : currentPlay->getRoleStatuses()) {
            displayRole(role, state);
        }
    }

    void STPVisualizerWidget::displayTactic(stp::Tactic *tactic) {
        if (!tactic) {
            updateContent << "None<br>";
            return;
        }

        updateContent << "Some(" << tactic->getName() << ") => " << getNameForStatus(tactic->getStatus()) << ":<br>";
        displaySkill(tactic->getCurrentSkill());
    }

    void STPVisualizerWidget::displayRole(stp::Role *role, stp::Status state) {
        updateContent << role->getName() << " ";
        auto& curBot = role->getCurrentRobot();
        if (!curBot) {
            updateContent << "None<br>";
            return;
        }

        auto& botView = curBot.value();
        if (!botView) {
            updateContent << "None<br>";
            return;
        }

        // todo -> colors
        updateContent << "Some(" << botView->getId() << ") => " << getNameForStatus(state) << "<br>";
        displayTactic(role->getCurrentTactic());
    }

    void STPVisualizerWidget::displaySkill(stp::Skill *skill) {
        if (!skill) {
            updateContent << "None<br>";
            return;
        }

        updateContent << "Some(" << skill->name() << ") => " << getNameForStatus(skill->getStatus()) << "<br>";
    }

    constexpr const char *STPVisualizerWidget::getNameForStatus(stp::Status status) {
        switch (status) {
            case stp::Status::Success:
                return "Success";
            case stp::Status::Failure:
                return "Failure";
            case stp::Status::Running:
                return "Running";
            case stp::Status::Waiting:
                return "Waiting";
            default:
                return "Invalid";
        }
    }

    void STPVisualizerWidget::outputStpData() {
        this->setText(QString::fromStdString(updateContent.str()));
    }
}  // namespace rtt::ai::interface