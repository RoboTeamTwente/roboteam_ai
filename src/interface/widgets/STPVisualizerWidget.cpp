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
    constexpr static const char *space = "&nbsp;";
    constexpr static const char *tab = "&nbsp;&nbsp;&nbsp;&nbsp;";

    STPVisualizerWidget::STPVisualizerWidget(MainWindow *parent) : QTextEdit(parent) {
        this->parent = parent;
        this->setReadOnly(true);
    }

    std::string_view STPVisualizerWidget::getColorForStatus(stp::Status status) {
        switch (status) {
            case stp::Status::Failure:
                return "<font color=\"Red\">";
            case stp::Status::Running:
                return "<font color=\"Green\">";  // dark green
            case stp::Status::Success:
                return "<font color=\"Lime\">";  // bright green
            case stp::Status::Waiting:
                return "<font color=\"Aqua\">";
            default:
                return "<font color=\"White\">";
        }
    }

    void STPVisualizerWidget::updateContents(stp::Play *currentPlay) {
        updateContent.str("");
        displayPlay(currentPlay);
    }

    void STPVisualizerWidget::displayPlay(stp::Play *currentPlay) {
        updateContent << "Play: ";
        if (!currentPlay) {
            updateContent << "None<br>";
            return;
        }
        updateContent << "Some(" << currentPlay->getName() << ")<br>" << tab;
        std::vector<std::pair<stp::Role*, stp::Status>> states = { currentPlay->getRoleStatuses().begin(), currentPlay->getRoleStatuses().end() };
        std::sort(states.begin(), states.end(), [](auto const& lhs, auto const& rhs) {
            auto firstRobot = lhs.first->getCurrentRobot();
            auto secondRobot = rhs.first->getCurrentRobot();

            if (!firstRobot || !firstRobot.value()) {
                return true;
            }
            if (!secondRobot || !firstRobot.value()) {
                return false;
            }

            return firstRobot.value()->getId() < secondRobot.value()->getId();
        });

        for (auto &elem : states) {
            auto&[role, state] = elem;
            displayRole(role, state, &elem == &*states.end());
        }
    }

    void STPVisualizerWidget::displayTactic(stp::Tactic *tactic, bool last) {
        if (!tactic) {
            updateContent << "None<br>" << tab << tab;
            return;
        }

        updateContent << "Some(" << tactic->getName() << ") => ";
        outputStatus(tactic->getStatus());
        updateContent << ":<br>" << tab << tab << tab;
        displaySkill(tactic->getCurrentSkill(), last);
    }

    void STPVisualizerWidget::displayRole(stp::Role *role, stp::Status state, bool last) {
        updateContent << role->getName() << " ";
        auto &curBot = role->getCurrentRobot();
        if (!curBot) {
            updateContent << "None<br>" << tab;
            return;
        }

        auto &botView = curBot.value();
        if (!botView) {
            updateContent << "None<br>" << tab;
            return;
        }

        parent->setPlayForRobot(role->getName(), botView->getId());

        if (robotDealer::RobotDealer::getKeeperID() == botView->getId()) {
            parent->setKeeperRole(role, state);
            return;
        }
        updateContent << "Some(" << botView->getId() << ") => ";
        outputStatus(state);
        updateContent << ":<br>" << tab << tab;
        displayTactic(role->getCurrentTactic(), last);
    }

    void STPVisualizerWidget::displaySkill(stp::Skill *skill, bool last) {
        if (!skill) {
            updateContent << "None<br>";
            return;
        }

        updateContent << "Some(" << skill->name() << ") => ";
        outputStatus(skill->getStatus());
        updateContent << "<br><br>";
        if (!last) {
            updateContent << tab;
        }
    }

    void STPVisualizerWidget::outputStpData() {
        this->setHtml(QString::fromStdString(updateContent.str()));
    }

    void STPVisualizerWidget::outputStatus(stp::Status status) {
        updateContent << getColorForStatus(status) << status << "</font>";
    }

    void STPVisualizerWidget::updateKeeperContents(stp::Role *pRole, stp::Status state) {
        updateContent.str("");
        updateContent << "Keeper Role:";
        displayRole(pRole, state, false);
        outputStpData();
    }
}  // namespace rtt::ai::interface