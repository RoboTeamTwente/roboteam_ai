/*
 *
 * This widget visualizes a behaviour (strategy) tree.
 *
 * Update contents updates the statuses, ticktimes and amount of ticks for each node.
 * The nodes in the tree are not refreshed until the whole tree is invalidated.
 *
 */

#include "interface/widgets/STPVisualizerWidget.h"

#include <QtWidgets/QLayoutItem>
#include <utilities/GameStateManager.hpp>

#include "interface/widgets/mainWindow.h"
#include "stp/Play.hpp"

namespace rtt::ai::interface {
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
    std::lock_guard lck{contentLock};
    updateContent.str("");
    displayPlay(currentPlay);
}

void STPVisualizerWidget::displayPlay(stp::Play *currentPlay) {
    updateContent << " Play: ";
    if (!currentPlay) {
        updateContent << "None<br>";
        return;
    }
    updateContent << currentPlay->getName() << tab;
    updateContent << "||" << tab << "Play Score: " << std::to_string(currentPlay->getLastScore()) << "<br>" << tab;
    std::vector<std::pair<stp::Role *, stp::Status>> states = {currentPlay->getRoleStatuses().begin(), currentPlay->getRoleStatuses().end()};
    // lhs and rhs are std::pair<stp::Role*, stp::Status>>, this function returns true if lhs < rhs (the robot id)
    std::sort(states.begin(), states.end(), [](auto const &lhs, auto const &rhs) {
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
        auto &[role, state] = elem;
        displayRole(role, state, &elem == &*states.end());
    }
}

void STPVisualizerWidget::displayTactic(stp::Tactic *tactic, bool last) {
    updateContent << "Tactic: ";
    if (!tactic) {
        updateContent << "None<br>" << tab << tab;
        return;
    }

    updateContent << tactic->getName() << " => ";
    outputStatus(tactic->getStatus());
    updateContent << ":<br>" << tab << tab << tab;
    displaySkill(tactic->getCurrentSkill(), last);
}

void STPVisualizerWidget::displayRole(stp::Role *role, stp::Status state, bool last, bool updatingForKeeper) {
    auto &curBot = role->getCurrentRobot();
    auto &botView = curBot.value();

    if (!updatingForKeeper && role->getName() == "keeper" && GameStateManager::getCurrentGameState().keeperId == botView->getId()) {
        parent->setKeeperRole(role, state);
        return;
    }
    updateContent << "Role: ";
    updateContent << role->getName() << " ";
    if (!curBot) {
        updateContent << "None<br>" << tab;
        return;
    }

    if (!botView) {
        updateContent << "None<br>" << tab;
        return;
    }

    parent->setPlayForRobot(role->getName(), botView->getId());

    updateContent << "Robot ID: " << botView->getId() << " => ";
    outputStatus(state);
    updateContent << ":<br>" << tab << tab;
    displayTactic(role->getCurrentTactic(), last);
}

void STPVisualizerWidget::displaySkill(stp::Skill *skill, bool last) {
    updateContent << "Skill: ";
    if (!skill) {
        updateContent << "None<br>";
        return;
    }

    updateContent << skill->getName() << " => ";
    outputStatus(skill->getStatus());
    updateContent << "<br><br>";
    if (!last) {
        updateContent << tab;
    }
}

void STPVisualizerWidget::outputStpData() {
    auto sliderPos = this->verticalScrollBar()->sliderPosition();
    setHtml(QString::fromStdString(updateContent.str()));
    this->verticalScrollBar()->setSliderPosition(sliderPos);
}

void STPVisualizerWidget::outputStatus(stp::Status status) { updateContent << getColorForStatus(status) << status << "</font>"; }

void STPVisualizerWidget::updateKeeperContents(stp::Role *pRole, stp::Status state) {
    std::lock_guard lck{contentLock};
    updateContent.str("");
    updateContent << "Keeper Role: <br>" << tab;
    displayRole(pRole, state, false, true);
}
}  // namespace rtt::ai::interface